#ifndef __FEMU_MEM_BACKEND
#define __FEMU_MEM_BACKEND

#include <infiniband/verbs.h>
#include <stdint.h>

typedef struct FemuRdmaCtx {
    struct ibv_context *ctx;
    struct ibv_pd      *pd;
    struct ibv_cq      *cq;

    struct ibv_qp      *qp1;
    struct ibv_qp      *qp2;

    int initialized;

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

} SsdDramBackend;

int init_dram_backend(SsdDramBackend **mbe, int64_t nbytes);
void free_dram_backend(SsdDramBackend *);

int backend_rw(SsdDramBackend *, QEMUSGList *, uint64_t *, bool);

#endif
