#ifndef __FEMU_DRAM_RDMA_H
#define __FEMU_DRAM_RDMA_H

#include "../nvme.h"
#include "dram.h"

int rdma_init_backend(SsdDramBackend *b);

int rdma_write_bounce_to_backend(SsdDramBackend *b, uint64_t backend_off, size_t len);
int rdma_read_backend_to_bounce(SsdDramBackend *b, uint64_t backend_off, size_t len);

#endif