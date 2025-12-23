#include "../nvme.h"
#include "dram.h"

static int femu_rdma_init_backend(SsdDramBackend *b);

/* Coperd: FEMU Memory Backend (mbe) for emulated SSD */

int init_dram_backend(SsdDramBackend **mbe, int64_t nbytes)
{
    fprintf(stderr, "FEMU DRAM backend init ENTERED\n");
    fflush(stderr);
    SsdDramBackend *b = *mbe = g_malloc0(sizeof(SsdDramBackend));

    b->size = nbytes;
    b->logical_space = g_malloc0(nbytes);

    b->enable_rdma = 1; 

    if (mlock(b->logical_space, nbytes) == -1) {
        femu_err("Failed to pin the memory backend to the host DRAM\n");
        g_free(b->logical_space);
        abort();
    }

    if (b->enable_rdma) {
    if (femu_rdma_init_backend(b) != 0) {
        femu_err("RDMA init failed, falling back to DMA\n");
        b->enable_rdma = 0;
    }
}


    return 0;
}

void free_dram_backend(SsdDramBackend *b)
{
    if (b->logical_space) {
        munlock(b->logical_space, b->size);
        g_free(b->logical_space);
    }
}

int backend_rw(SsdDramBackend *b, QEMUSGList *qsg, uint64_t *lbal, bool is_write)
{
    int sg_cur_index = 0;
    dma_addr_t sg_cur_byte = 0;
    dma_addr_t cur_addr, cur_len;
    uint64_t mb_oft = lbal[0];
    void *mb = b->logical_space;

    DMADirection dir = DMA_DIRECTION_FROM_DEVICE;

    if (is_write) {
        dir = DMA_DIRECTION_TO_DEVICE;
    }

    while (sg_cur_index < qsg->nsg) {
        cur_addr = qsg->sg[sg_cur_index].base + sg_cur_byte;
        cur_len = qsg->sg[sg_cur_index].len - sg_cur_byte;
        if (dma_memory_rw(qsg->as, cur_addr, mb + mb_oft, cur_len, dir, MEMTXATTRS_UNSPECIFIED)) {
            femu_err("dma_memory_rw error\n");
        }

        sg_cur_byte += cur_len;
        if (sg_cur_byte == qsg->sg[sg_cur_index].len) {
            sg_cur_byte = 0;
            ++sg_cur_index;
        }

        if (b->femu_mode == FEMU_OCSSD_MODE) {
            mb_oft = lbal[sg_cur_index];
        } else if (b->femu_mode == FEMU_BBSSD_MODE ||
                   b->femu_mode == FEMU_NOSSD_MODE ||
                   b->femu_mode == FEMU_ZNSSD_MODE) {
            mb_oft += cur_len;
        } else {
            assert(0);
        }
    }

    qemu_sglist_destroy(qsg);

    return 0;
}


static int femu_rdma_init_backend(SsdDramBackend *b)
{

    fprintf(stderr, "[RDMA] init start\n");
    fflush(stderr);


    struct ibv_device **dev_list;
    int num_devs;

    dev_list = ibv_get_device_list(&num_devs);
    if (!dev_list || num_devs == 0) {
        femu_err("No RDMA devices found\n");
        return -1;
    }

    b->rdma.ctx = ibv_open_device(dev_list[0]);
    fprintf(stderr, "[RDMA] device opened\n");

    ibv_free_device_list(dev_list);

    if (!b->rdma.ctx)
        return -1;

    b->rdma.pd = ibv_alloc_pd(b->rdma.ctx);
    fprintf(stderr, "[RDMA] PD allocated\n");

    if (!b->rdma.pd)
        return -1;

    b->rdma.cq = ibv_create_cq(b->rdma.ctx, 16, NULL, NULL, 0);
    if (!b->rdma.cq)
        return -1;

    /* create qp1 and qp2 */

    /* REGISTER BACKEND DRAM BUFFER */
    b->mr_backend = ibv_reg_mr(
        b->rdma.pd,
        b->logical_space,
        b->size,
        IBV_ACCESS_LOCAL_WRITE |
        IBV_ACCESS_REMOTE_READ |
        IBV_ACCESS_REMOTE_WRITE
    );

    fprintf(stderr,
    "[RDMA] MR registered addr=%p size=%ld lkey=%u rkey=%u\n",
    b->logical_space,
    b->size,
    b->mr_backend->lkey,
    b->mr_backend->rkey
);


    if (!b->mr_backend) {
        femu_err("Failed to register backend MR\n");
        return -1;
    }

    b->rdma.initialized = 1;


    return 0;
}