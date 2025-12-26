#ifndef __FEMU_MEM_BACKEND
#define __FEMU_MEM_BACKEND
#define FEMU_BOUNCE_SIZE (128 * 1024) 

#include <infiniband/verbs.h>
#include <stdint.h>
#include "../nvme.h"

typedef struct FemuRdmaCtx {
    struct ibv_context *ctx;
    struct ibv_pd      *pd;
    struct ibv_cq      *cq;

    struct ibv_qp      *qp1;
    struct ibv_qp      *qp2;

    int initialized;

    uint64_t rdma_write_cnt;
    uint64_t rdma_read_cnt;

} FemuRdmaCtx;


/* DRAM backend SSD address space */
typedef struct SsdDramBackend {
    void    *logical_space;
    int64_t size; /* in bytes */
    int     femu_mode;

    //rdma
    int enable_rdma;
    FemuRdmaCtx rdma;

    struct ibv_mr *mr_backend;

    void          *bounce_buf;
    size_t         bounce_size;
    struct ibv_mr *mr_bounce;

    uint64_t rdma_wr_seq;

} SsdDramBackend;

int init_dram_backend(SsdDramBackend **mbe, int64_t nbytes);
void free_dram_backend(SsdDramBackend *);
int backend_rw(SsdDramBackend *, QEMUSGList *, uint64_t *, bool);

#endif
