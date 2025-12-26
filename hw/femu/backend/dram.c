#include "../nvme.h"
#include "dram.h"
#include "dram_rdma.h"

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