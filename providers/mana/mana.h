/* SPDX-License-Identifier: GPL-2.0 OR Linux-OpenIB */
/*
 * Copyright (c) 2022, Microsoft Corporation. All rights reserved.
 */

#ifndef _MANA_H_
#define _MANA_H_

#include "manadv.h"

#define COMP_ENTRY_SIZE 64
#define MANA_IB_TOEPLITZ_HASH_KEY_SIZE_IN_BYTES 40

#define DMA_OOB_SIZE 8

#define INLINE_OOB_SMALL_SIZE 8
#define INLINE_OOB_LARGE_SIZE 24

#define GDMA_WQE_ALIGNMENT_UNIT_SIZE 32

/* The size of a SGE in WQE */
#define SGE_SIZE 16

#define DOORBELL_PAGE_SIZE 4096
#define MANA_PAGE_SIZE 4096

static inline uint32_t align_hw_size(uint32_t size)
{
	size = roundup_pow_of_two(size);
	return align(size, MANA_PAGE_SIZE);
}

static inline uint32_t get_wqe_size(uint32_t sge)
{
	uint32_t wqe_size = sge * SGE_SIZE + DMA_OOB_SIZE + INLINE_OOB_SMALL_SIZE;

	return align(wqe_size, GDMA_WQE_ALIGNMENT_UNIT_SIZE);
}

struct mana_context {
	struct verbs_context ibv_ctx;
	struct manadv_ctx_allocators extern_alloc;
	void *db_page;
};

struct mana_rwq_ind_table {
	struct ibv_rwq_ind_table ib_ind_table;

	uint32_t ind_tbl_size;
	struct ibv_wq **ind_tbl;
};

struct mana_qp {
	struct verbs_qp ibqp;

	void *send_buf;
	uint32_t send_buf_size;

	int send_wqe_count;

	uint32_t sqid;
	uint32_t tx_vp_offset;
};

struct mana_wq {
	struct ibv_wq ibwq;

	void *buf;
	uint32_t buf_size;

	uint32_t wqe;
	uint32_t sge;

	uint32_t wqid;
};

struct mana_cq {
	struct ibv_cq ibcq;
	uint32_t cqe;
	uint32_t cqid;
	void *buf;

	pthread_spinlock_t lock;
	uint32_t head;
	uint32_t last_armed_head;
	uint32_t ready_wcs;
	void *db_page;
	/* list of qp's that use this cq for send completions */
	struct list_head send_qp_list;
	/* list of qp's that use this cq for recv completions */
	struct list_head recv_qp_list;
	bool buf_external;
};

struct mana_device {
	struct verbs_device verbs_dev;
};

struct mana_pd {
	struct ibv_pd ibv_pd;
	struct mana_pd *mprotection_domain;
};

struct mana_parent_domain {
	struct mana_pd mpd;
	void *pd_context;
};

struct mana_context *to_mctx(struct ibv_context *ibctx);

void *mana_alloc_mem(uint32_t size);

int mana_query_device_ex(struct ibv_context *context,
			 const struct ibv_query_device_ex_input *input,
			 struct ibv_device_attr_ex *attr, size_t attr_size);

int mana_query_port(struct ibv_context *context, uint8_t port,
		    struct ibv_port_attr *attr);

struct ibv_pd *mana_alloc_pd(struct ibv_context *context);
struct ibv_pd *
mana_alloc_parent_domain(struct ibv_context *context,
			 struct ibv_parent_domain_init_attr *attr);

int mana_dealloc_pd(struct ibv_pd *pd);

struct ibv_mr *mana_reg_mr(struct ibv_pd *pd, void *addr, size_t length,
			   uint64_t hca_va, int access);

int mana_dereg_mr(struct verbs_mr *vmr);

struct ibv_cq *mana_create_cq(struct ibv_context *context, int cqe,
			      struct ibv_comp_channel *channel,
			      int comp_vector);

int mana_destroy_cq(struct ibv_cq *cq);

int mana_poll_cq(struct ibv_cq *ibcq, int nwc, struct ibv_wc *wc);

struct ibv_wq *mana_create_wq(struct ibv_context *context,
			      struct ibv_wq_init_attr *attr);

int mana_destroy_wq(struct ibv_wq *wq);
int mana_modify_wq(struct ibv_wq *ibwq, struct ibv_wq_attr *attr);

struct ibv_rwq_ind_table *
mana_create_rwq_ind_table(struct ibv_context *context,
			  struct ibv_rwq_ind_table_init_attr *init_attr);

int mana_destroy_rwq_ind_table(struct ibv_rwq_ind_table *rwq_ind_table);

struct ibv_qp *mana_create_qp(struct ibv_pd *pd, struct ibv_qp_init_attr *attr);

struct ibv_qp *mana_create_qp_ex(struct ibv_context *context,
				 struct ibv_qp_init_attr_ex *attr);

int mana_modify_qp(struct ibv_qp *qp, struct ibv_qp_attr *attr, int attr_mask);

int mana_destroy_qp(struct ibv_qp *ibqp);

int mana_post_recv(struct ibv_qp *ibqp, struct ibv_recv_wr *wr,
		   struct ibv_recv_wr **bad);

int mana_post_send(struct ibv_qp *ibqp, struct ibv_send_wr *wr,
		   struct ibv_send_wr **bad);

#endif
