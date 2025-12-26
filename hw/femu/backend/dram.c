#include "../nvme.h"
#include "dram.h"

static int rdma_init_backend(SsdDramBackend *b);
static int rdma_qp_connect_loopback(SsdDramBackend *b, uint8_t port_num);
static int rdma_write_bounce_to_backend(SsdDramBackend *b, uint64_t backend_off, size_t len);
static int rdma_read_backend_to_bounce(SsdDramBackend *b, uint64_t backend_off, size_t len);

/* Coperd: FEMU Memory Backend (mbe) for emulated SSD */

int init_dram_backend(SsdDramBackend **mbe, int64_t nbytes)
{
    fprintf(stderr, "FEMU DRAM backend init ENTERED\n");
    fflush(stderr);
    SsdDramBackend *b = *mbe = g_malloc0(sizeof(SsdDramBackend));

    b->size = nbytes;
    b->logical_space = g_malloc0(nbytes);

    b->enable_rdma = 1;

    b->bounce_size = FEMU_BOUNCE_SIZE;
    b->bounce_buf = g_malloc0(b->bounce_size);

    if (!b->bounce_buf)
    {
        femu_err("Failed to allocate bounce buffer\n");
        abort();
    }

    if (mlock(b->logical_space, nbytes) == -1)
    {
        femu_err("Failed to pin the memory backend to the host DRAM\n");
        g_free(b->logical_space);
        abort();
    }

    if (b->enable_rdma)
    {
        if (rdma_init_backend(b) != 0)
        {
            femu_err("RDMA init failed, falling back to DMA\n");
            b->enable_rdma = 0;
        }
    }

    return 0;
}

void free_dram_backend(SsdDramBackend *b)
{
    if (b->logical_space)
    {
        munlock(b->logical_space, b->size);
        g_free(b->logical_space);
    }

    if (b->mr_bounce)
    {
        ibv_dereg_mr(b->mr_bounce);
    }

    if (b->bounce_buf)
    {
        g_free(b->bounce_buf);
    }
}

int backend_rw(SsdDramBackend *b, QEMUSGList *qsg, uint64_t *lbal, bool is_write)
{

    fprintf(stderr,
        "[RDMA-CHECK] enable_rdma=%d initialized=%d is_write=%d\n",
        b->enable_rdma,
        b->rdma.initialized,
        is_write);


    int sg_cur_index = 0;
    dma_addr_t sg_cur_byte = 0;
    dma_addr_t cur_addr, cur_len;
    uint64_t mb_oft = lbal[0];
    uint8_t *backend = b->logical_space;
    uint8_t *bounce = b->bounce_buf;

    while (sg_cur_index < qsg->nsg)
    {
        cur_addr = qsg->sg[sg_cur_index].base + sg_cur_byte;
        cur_len = qsg->sg[sg_cur_index].len - sg_cur_byte;

        // checks on bounce
        if (cur_len > b->bounce_size)
        {
            fprintf(stderr,
                    "[RDMA] BUG: cur_len=%zu > bounce_size=%zu\n",
                    cur_len, b->bounce_size);
            return -1;
        }

        if (mb_oft + cur_len > (uint64_t)b->size)
        {
            fprintf(stderr,
                    "[RDMA] BUG: backend OOB: off=%lu len=%zu size=%ld\n",
                    (unsigned long)mb_oft, cur_len, (long)b->size);
            return -1;
        }

        if (is_write)
        {
            /* Guest → Bounce */
            dma_memory_rw(qsg->as, cur_addr,
                          bounce, cur_len,
                          DMA_DIRECTION_TO_DEVICE,
                          MEMTXATTRS_UNSPECIFIED);

            /* Bounce → Backend */
            if (b->enable_rdma)
            {
                if (rdma_write_bounce_to_backend(b, mb_oft, cur_len) != 0)
                {
                    return -1;
                }
            }
            else
            {
                memcpy(backend + mb_oft, bounce, cur_len);
            }

            
        }
        else
        {
            /* Backend → Bounce */
            if (b->enable_rdma)
            {
                if (rdma_read_backend_to_bounce(b, mb_oft, cur_len) != 0)
                {
                    return -1;
                }
            }
            else
            {
                memcpy(bounce, backend + mb_oft, cur_len);
            }

            /* Bounce → Guest */
            dma_memory_rw(qsg->as, cur_addr,
                          bounce, cur_len,
                          DMA_DIRECTION_FROM_DEVICE,
                          MEMTXATTRS_UNSPECIFIED);
        }

        if (b->enable_rdma)
        {
            fprintf(stderr,
                    "[RDMA] bounce used addr=%p len=%lu\n",
                    bounce, cur_len);
        }

        sg_cur_byte += cur_len;
        if (sg_cur_byte == qsg->sg[sg_cur_index].len)
        {
            sg_cur_byte = 0;
            ++sg_cur_index;
        }

        if (b->femu_mode == FEMU_OCSSD_MODE)
        {
            mb_oft = lbal[sg_cur_index];
        }
        else if (b->femu_mode == FEMU_BBSSD_MODE ||
                 b->femu_mode == FEMU_NOSSD_MODE ||
                 b->femu_mode == FEMU_ZNSSD_MODE)
        {
            mb_oft += cur_len;
        }
        else
        {
            assert(0);
        }

        if ((b->rdma.rdma_write_cnt & 0xFF) == 0)
        {
            fprintf(stderr, "[RDMA] writes=%lu reads=%lu\n",
                    b->rdma.rdma_write_cnt,
                    b->rdma.rdma_read_cnt);
        }
    }

    qemu_sglist_destroy(qsg);

    return 0;
}

static int rdma_init_backend(SsdDramBackend *b)
{
    fprintf(stderr, "[RDMA] init start\n");
    fflush(stderr);

    struct ibv_device **dev_list;
    int num_devs;

    dev_list = ibv_get_device_list(&num_devs);
    if (!dev_list || num_devs == 0)
    {
        femu_err("No RDMA devices found\n");
        return -1;
    }

    b->rdma.ctx = ibv_open_device(dev_list[0]);
    ibv_free_device_list(dev_list);
    if (!b->rdma.ctx)
    {
        femu_err("ibv_open_device failed\n");
        return -1;
    }
    fprintf(stderr, "[RDMA] device opened\n");

    b->rdma.pd = ibv_alloc_pd(b->rdma.ctx);
    if (!b->rdma.pd)
    {
        femu_err("ibv_alloc_pd failed\n");
        return -1;
    }
    fprintf(stderr, "[RDMA] PD allocated\n");

    b->rdma.cq = ibv_create_cq(b->rdma.ctx, 256, NULL, NULL, 0);
    if (!b->rdma.cq)
    {
        femu_err("ibv_create_cq failed\n");
        return -1;
    }

    // Create QPs
    struct ibv_qp_init_attr qpia;
    memset(&qpia, 0, sizeof(qpia));
    qpia.send_cq = b->rdma.cq;
    qpia.recv_cq = b->rdma.cq;
    qpia.qp_type = IBV_QPT_RC;
    qpia.cap.max_send_wr = 256;
    qpia.cap.max_recv_wr = 16;
    qpia.cap.max_send_sge = 1;
    qpia.cap.max_recv_sge = 1;

    b->rdma.qp1 = ibv_create_qp(b->rdma.pd, &qpia);
    b->rdma.qp2 = ibv_create_qp(b->rdma.pd, &qpia);
    if (!b->rdma.qp1 || !b->rdma.qp2)
    {
        femu_err("ibv_create_qp failed\n");
        return -1;
    }

    // Connect QPs
    if (rdma_qp_connect_loopback(b, /*port_num=*/1) != 0)
    {
        femu_err("RDMA QP connect loopback failed\n");
        return -1;
    }

    // Register backend MR
    b->mr_backend = ibv_reg_mr(
        b->rdma.pd,
        b->logical_space,
        b->size,
        IBV_ACCESS_LOCAL_WRITE |
            IBV_ACCESS_REMOTE_READ |
            IBV_ACCESS_REMOTE_WRITE);

    if (!b->mr_backend)
    {
        femu_err("Failed to register backend MR\n");
        return -1;
    }

    fprintf(stderr, "[RDMA] backend MR addr=%p size=%ld lkey=%u rkey=%u\n",
            b->logical_space, b->size,
            b->mr_backend->lkey, b->mr_backend->rkey);
    fflush(stderr);

    // Register bounce MR
    b->mr_bounce = ibv_reg_mr(
        b->rdma.pd,
        b->bounce_buf,
        b->bounce_size,
        IBV_ACCESS_LOCAL_WRITE |
            IBV_ACCESS_REMOTE_READ |
            IBV_ACCESS_REMOTE_WRITE);

    if (!b->mr_bounce)
    {
        femu_err("Failed to register bounce MR\n");
        return -1;
    }

    fprintf(stderr, "[RDMA] bounce MR addr=%p size=%zu lkey=%u rkey=%u\n",
            b->bounce_buf, b->bounce_size,
            b->mr_bounce->lkey, b->mr_bounce->rkey);
    fflush(stderr);

    b->rdma.initialized = 1;
    return 0;
}

static int qp_to_init(struct ibv_qp *qp, uint8_t port_num)
{
    struct ibv_qp_attr attr = {
        .qp_state = IBV_QPS_INIT,
        .pkey_index = 0,
        .port_num = port_num,
        .qp_access_flags = IBV_ACCESS_REMOTE_READ | IBV_ACCESS_REMOTE_WRITE,
    };

    int flags = IBV_QP_STATE | IBV_QP_PKEY_INDEX | IBV_QP_PORT | IBV_QP_ACCESS_FLAGS;
    return ibv_modify_qp(qp, &attr, flags);
}

static int qp_to_rtr(struct ibv_qp *qp,
                     uint32_t remote_qpn,
                     uint32_t remote_psn,
                     uint8_t port_num,
                     int is_roce,
                     uint16_t dlid,
                     int sgid_index,
                     union ibv_gid dgid)
{
    struct ibv_qp_attr attr;
    memset(&attr, 0, sizeof(attr));

    attr.qp_state = IBV_QPS_RTR;
    attr.path_mtu = IBV_MTU_1024;
    attr.dest_qp_num = remote_qpn;
    attr.rq_psn = remote_psn;
    attr.max_dest_rd_atomic = 1;
    attr.min_rnr_timer = 12;

    attr.ah_attr.is_global = is_roce ? 1 : 0;
    attr.ah_attr.dlid = is_roce ? 0 : dlid;
    attr.ah_attr.sl = 0;
    attr.ah_attr.src_path_bits = 0;
    attr.ah_attr.port_num = port_num;

    if (is_roce)
    {
        attr.ah_attr.grh.dgid = dgid;
        attr.ah_attr.grh.sgid_index = sgid_index;
        attr.ah_attr.grh.hop_limit = 1;
        attr.ah_attr.grh.traffic_class = 0;
        attr.ah_attr.grh.flow_label = 0;
    }

    int flags = IBV_QP_STATE |
                IBV_QP_AV |
                IBV_QP_PATH_MTU |
                IBV_QP_DEST_QPN |
                IBV_QP_RQ_PSN |
                IBV_QP_MAX_DEST_RD_ATOMIC |
                IBV_QP_MIN_RNR_TIMER;

    return ibv_modify_qp(qp, &attr, flags);
}

static int qp_to_rts(struct ibv_qp *qp, uint32_t local_psn)
{
    struct ibv_qp_attr attr = {
        .qp_state = IBV_QPS_RTS,
        .sq_psn = local_psn,
        .timeout = 14,
        .retry_cnt = 7,
        .rnr_retry = 7,
        .max_rd_atomic = 1,
    };

    int flags = IBV_QP_STATE | IBV_QP_SQ_PSN | IBV_QP_TIMEOUT |
                IBV_QP_RETRY_CNT | IBV_QP_RNR_RETRY | IBV_QP_MAX_QP_RD_ATOMIC;

    return ibv_modify_qp(qp, &attr, flags);
}

static int rdma_qp_connect_loopback(SsdDramBackend *b, uint8_t port_num)
{
    if (!b || !b->rdma.ctx || !b->rdma.qp1 || !b->rdma.qp2)
    {
        femu_err("RDMA connect called before QPs/ctx exist\n");
        return -1;
    }

    struct ibv_port_attr port_attr;
    if (ibv_query_port(b->rdma.ctx, port_num, &port_attr) != 0)
    {
        femu_err("ibv_query_port failed\n");
        return -1;
    }

    if (port_attr.state != IBV_PORT_ACTIVE)
    {
        femu_err("RDMA port not ACTIVE (state=%d)\n", port_attr.state);
        return -1;
    }

    int is_roce = (port_attr.link_layer == IBV_LINK_LAYER_ETHERNET);
    uint16_t lid = port_attr.lid;

    int sgid_index = 0;
    union ibv_gid gid;
    memset(&gid, 0, sizeof(gid));

    if (is_roce)
    {
        if (ibv_query_gid(b->rdma.ctx, port_num, sgid_index, &gid) != 0)
        {
            femu_err("ibv_query_gid failed (port=%u sgid_index=%d)\n", port_num, sgid_index);
            return -1;
        }
    }
    else
    {

        if (lid == 0)
        {
            femu_err("[WARN] IB link_layer but LID=0; check fabric/SM\n");
        }
    }

    // PSNs
    srand48((long)time(NULL));
    uint32_t psn1 = lrand48() & 0xffffff;
    uint32_t psn2 = lrand48() & 0xffffff;

    // INIT for both
    if (qp_to_init(b->rdma.qp1, port_num) != 0)
    {
        femu_err("qp1 -> INIT failed\n");
        return -1;
    }
    if (qp_to_init(b->rdma.qp2, port_num) != 0)
    {
        femu_err("qp2 -> INIT failed\n");
        return -1;
    }

    // RTR
    if (qp_to_rtr(b->rdma.qp1,
                  b->rdma.qp2->qp_num,
                  psn2,
                  port_num,
                  is_roce,
                  lid,
                  sgid_index,
                  gid) != 0)
    {
        femu_err("qp1 -> RTR failed\n");
        return -1;
    }

    if (qp_to_rtr(b->rdma.qp2,
                  b->rdma.qp1->qp_num,
                  psn1,
                  port_num,
                  is_roce,
                  lid,
                  sgid_index,
                  gid) != 0)
    {
        femu_err("qp2 -> RTR failed\n");
        return -1;
    }

    // RTS
    if (qp_to_rts(b->rdma.qp1, psn1) != 0)
    {
        femu_err("qp1 -> RTS failed\n");
        return -1;
    }
    if (qp_to_rts(b->rdma.qp2, psn2) != 0)
    {
        femu_err("qp2 -> RTS failed\n");
        return -1;
    }

    fprintf(stderr,
            "[RDMA] QPs connected loopback: link_layer=%s port=%u lid=%u qp1=%u qp2=%u psn1=%u psn2=%u sgid_index=%d\n",
            is_roce ? "ETH(RoCE)" : "IB",
            port_num,
            lid,
            b->rdma.qp1->qp_num,
            b->rdma.qp2->qp_num,
            psn1,
            psn2,
            sgid_index);
    fflush(stderr);

    return 0;
}

static int rdma_poll_one_wc(struct ibv_cq *cq, uint64_t expect_wr_id)
{
    struct ibv_wc wc;
    int ne;

    do
    {
        ne = ibv_poll_cq(cq, 1, &wc);
    } while (ne == 0);

    if (ne < 0)
    {
        femu_err("ibv_poll_cq failed ne=%d\n", ne);
        return -1;
    }
    if (wc.status != IBV_WC_SUCCESS)
    {
        femu_err("RDMA WC error: status=%s (%d) vendor_err=%u wr_id=%lu\n",
                 ibv_wc_status_str(wc.status), wc.status, wc.vendor_err,
                 (unsigned long)wc.wr_id);
        return -1;
    }
    if (wc.wr_id != expect_wr_id)
    {
        femu_err("RDMA WC mismatch: got wr_id=%lu expect=%lu\n",
                 (unsigned long)wc.wr_id, (unsigned long)expect_wr_id);
        return -1;
    }
    return 0;
}

static int rdma_write_bounce_to_backend(SsdDramBackend *b,
                                        uint64_t backend_off,
                                        size_t len)
{
    if (!b->enable_rdma || !b->rdma.initialized)
    {
        memcpy((uint8_t *)b->logical_space + backend_off, b->bounce_buf, len);
        return 0;
    }

    struct ibv_sge sge = {
        .addr = (uintptr_t)b->bounce_buf,
        .length = (uint32_t)len,
        .lkey = b->mr_bounce->lkey,
    };

    struct ibv_send_wr wr, *bad = NULL;
    memset(&wr, 0, sizeof(wr));

    uint64_t wr_id = (b->rdma_wr_seq++ << 1) | 1; // WRITE tag
    wr.wr_id = wr_id;

    fprintf(stderr, "[RDMA] WRITE off=%lu len=%zu wr_id=%lu\n",
            backend_off, len, wr_id);

    wr.sg_list = &sge;
    wr.num_sge = 1;
    wr.opcode = IBV_WR_RDMA_WRITE;
    wr.send_flags = IBV_SEND_SIGNALED; /* wait for completion */

    wr.wr.rdma.remote_addr = (uintptr_t)((uint8_t *)b->logical_space + backend_off);
    wr.wr.rdma.rkey = b->mr_backend->rkey;

    if (ibv_post_send(b->rdma.qp1, &wr, &bad) != 0)
    {
        fprintf(stderr, "[FEMU] Err: "
                        "ibv_post_send(WRITE) failed\n");
        return -1;
    }

    int ret = rdma_poll_one_wc(b->rdma.cq, wr_id);
    if (ret == 0)
    {
        b->rdma.rdma_write_cnt++;
    }

    return ret;
}

static int rdma_read_backend_to_bounce(SsdDramBackend *b,
                                       uint64_t backend_off,
                                       size_t len)
{
    if (!b->enable_rdma || !b->rdma.initialized)
    {
        memcpy(b->bounce_buf, (uint8_t *)b->logical_space + backend_off, len);
        return 0;
    }

    struct ibv_sge sge = {
        .addr = (uintptr_t)b->bounce_buf,
        .length = (uint32_t)len,
        .lkey = b->mr_bounce->lkey,
    };

    struct ibv_send_wr wr, *bad = NULL;
    memset(&wr, 0, sizeof(wr));

    uint64_t wr_id = (b->rdma_wr_seq++ << 1) | 0; // READ tag
    wr.wr_id = wr_id;

    fprintf(stderr, "[RDMA] READ off=%lu len=%zu wr_id=%lu\n",
            backend_off, len, wr_id);


    wr.sg_list = &sge;
    wr.num_sge = 1;
    wr.opcode = IBV_WR_RDMA_READ;
    wr.send_flags = IBV_SEND_SIGNALED; /* bounce is valid before DMA to guest */

    wr.wr.rdma.remote_addr = (uintptr_t)((uint8_t *)b->logical_space + backend_off);
    wr.wr.rdma.rkey = b->mr_backend->rkey;

    if (ibv_post_send(b->rdma.qp1, &wr, &bad) != 0)
    {
        fprintf(stderr, "[FEMU] Err: "
                        "ibv_post_send(READ) failed\n");
        return -1;
    }

    int ret = rdma_poll_one_wc(b->rdma.cq, wr_id);
    if (ret == 0)
    {
        b->rdma.rdma_read_cnt++;
    }

    return ret;
}
