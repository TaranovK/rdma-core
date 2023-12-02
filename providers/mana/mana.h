/* SPDX-License-Identifier: GPL-2.0 OR Linux-OpenIB */
/*
 * Copyright (c) 2022, Microsoft Corporation. All rights reserved.
 */

#ifndef _MANA_H_
#define _MANA_H_

#include "manadv.h"
#include <ccan/list.h>
#include <ccan/minmax.h>

#define COMP_ENTRY_SIZE 64
#define MANA_IB_TOEPLITZ_HASH_KEY_SIZE_IN_BYTES 40

#define DMA_OOB_SIZE 8

#define INLINE_OOB_SMALL_SIZE 8
#define INLINE_OOB_LARGE_SIZE 24

#define GDMA_WQE_ALIGNMENT_UNIT_SIZE 32

#define GDMA_WQE_ALIGNMENT_MASK  (GDMA_WQE_ALIGNMENT_UNIT_SIZE - 1)
#define MAX_TX_WQE_SIZE 512
#define MAX_RX_WQE_SIZE 256
/* The size of a SGE in WQE */
#define SGE_SIZE 16

#define DOORBELL_PAGE_SIZE 4096
#define MANA_PAGE_SIZE 4096

#define MANA_IB_MAX_DEST_RD_ATOMIC 16
#define MANA_IB_RESPONSE_WQE_SIZE 32
#define MANA_IB_MAX_RD_ATOMIC 1
#define MAKE_TAG(a, b, c, d) \
    (((uint32_t)(d) << 24) | ((c) << 16) | ((b) << 8) | (a))
#define RNIC_ROLLBACK_SHARED_MEM_SIG MAKE_TAG('R', 'L', 'B', 'K')

static inline int align_next_power2(int size)
{
	int val = 1;

	while (val < size)
		val <<= 1;

	return val;
}

static inline int align_hw_size(int size)
{
	size = align(size, MANA_PAGE_SIZE);
	return align_next_power2(size);
}

static inline int get_wqe_size(int sge)
{
	int wqe_size = sge * SGE_SIZE + DMA_OOB_SIZE + INLINE_OOB_SMALL_SIZE;

	return align(wqe_size, GDMA_WQE_ALIGNMENT_UNIT_SIZE);
}

static inline int get_large_wqe_size(int sge)
{
	int wqe_size = sge * SGE_SIZE + DMA_OOB_SIZE + INLINE_OOB_LARGE_SIZE;

	return align(wqe_size, GDMA_WQE_ALIGNMENT_UNIT_SIZE);
}

struct shadow_wqe_header {
	uint16_t opcode;
	uint16_t flags;
	uint32_t vendor_error_code;
	uint64_t wr_id;
};

struct rc_sq_shadow_wqe {
	struct  shadow_wqe_header header;
	union {
		struct {
			uint32_t start_psn;
			uint32_t count_packets;
			uint32_t unmasked_queue_offset;
			uint32_t read_unmasked_queue_offset;
			uint8_t posted_wqe_size;
			uint8_t read_posted_wqe_size;
			uint8_t preceding_fmr_sync_wqe_size;
			uint64_t posting_time;
		} psn_wqe;
	};
};

struct rc_rq_shadow_wqe {
	struct shadow_wqe_header header;
	uint32_t unmasked_q_offset;
	uint8_t posted_wqe_size;
	/* Data filled in by completion */
	struct {
		uint32_t byte_len;
		uint32_t psn;
		uint32_t imm_or_rkey;
	} compl;
};

struct shadow_queue {
	/* Unmasked producer index, Incremented on wqe posting */
	volatile uint64_t prod_idx;
	/* Unmasked consumer index, Incremented on cq  polling */
	volatile uint64_t cons_idx;
	/* Unmasked index of next-to-complete (from HW) shadow WQE */
	volatile uint64_t next_to_complete_idx;
	/* queue size in wqes */
	uint32_t length;
	/* distance between elements in bytes */
	uint32_t stride;
	/* ring buffer holding wqes */
	void *buffer;
	/* queue size in bytes */
	uint64_t size;
	uint32_t index_mask;
};

enum rc_queue_type {
	RC_RECV_QUEUE_REQUESTER = 0,
	RC_RECV_QUEUE_RESPONDER,
	RC_SEND_QUEUE_REQUESTER,
	RC_SEND_QUEUE_RESPONDER,
	RC_QUEUE_TYPE_MAX,
};

enum completion_type {
	COMP_TYPE_INVALID = 0,
	COMP_TYPE_SEND = BIT(0),
	COMP_TYPE_RECV = BIT(1),
	COMP_TYPE_SENDRECV = (COMP_TYPE_SEND | COMP_TYPE_RECV),
};

struct mana_ib_rollback_shared_mem {
	uint32_t signature;
	uint32_t size;
	uint32_t left_offset;
	uint32_t right_offset;
};

enum mana_ctx_flags {
	MANA_CTX_FLAG_FORCE_RNIC = 1,
};

struct mana_context {
	struct verbs_context ibv_ctx;
	struct manadv_ctx_allocators extern_alloc;
	void *db_page;
	uint32_t flags;
	struct mana_qp **lookup_table;
};

struct mana_rwq_ind_table {
	struct ibv_rwq_ind_table ib_ind_table;

	uint32_t ind_tbl_size;
	struct ibv_wq **ind_tbl;
};

struct mana_ib_raw_qp {
	void *send_buf;
	uint32_t send_buf_size;
	int send_wqe_count;
	uint32_t sqid;
	uint32_t tx_vp_offset;
};

struct mana_gdma_queue {
	void *buffer;
	uint32_t wqe_cnt;// in entries
	uint32_t size;// in bytes + rollback sh mem struct
	uint32_t wq_size;
	uint32_t id;
	uint32_t head;
	uint32_t tail;
	uint64_t *wrid;
};

struct mana_ib_rc_qp {
	struct mana_gdma_queue queues[RC_QUEUE_TYPE_MAX];

	uint32_t ssn;
	uint32_t sq_psn; //next to post psn
	uint32_t sq_highest_completed_psn; //The highest PSN we've seen in armed-completion CQE
	uint32_t sq_last_armed_psn; //The PSN we most recently armed for CQE generation
	uint64_t next_to_complete_psn_shadow_wqe_idx;
	uint32_t rq_highest_completed_psn; //The highest PSN we've seen in a receive HW CQE
	uint32_t dup_cqe_cnt; //Count of HW CQEs that were not strictly greater than the highest seen ACK PSN
	uint32_t no_wqe_completed_hw_cqe_cnt; //Count of HW CQEs that did not complete a shadow WQE
};

struct mana_qp {
	struct verbs_qp ibqp;

	struct list_node send_cq_node; /* cq used for send */
	struct list_node recv_cq_node; /* cq used for recv */

	struct shadow_queue shadow_sq;
	struct shadow_queue shadow_rq;

	union {
		struct mana_ib_raw_qp raw_qp;
		struct mana_ib_rc_qp rc_qp;
	};
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
	struct verbs_cq ibcq;
	uint32_t cqe;
	void *buf;
	uint32_t buf_size;
	uint32_t cqid;

	/* list of qp's that use this cq for send completions */
	struct list_head send_qp_list;
	/* list of qp's that use this cq for recv completions */
	struct list_head recv_qp_list;
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

static inline uint32_t decrement_psn(uint32_t psn)
{
	return (psn - 1) & 0xFFFFFF;
}

static inline struct mana_qp *to_mana_qp(struct ibv_qp *ibqp)
{
	return container_of(ibqp, struct mana_qp, ibqp.qp);
}

static inline struct mana_cq *to_mana_cq(struct ibv_cq *ibcq)
{
	return container_of(ibcq, struct mana_cq, ibcq.cq);
}

struct mana_context *to_mctx(struct ibv_context *ibctx);

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

#endif
