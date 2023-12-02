// SPDX-License-Identifier: GPL-2.0 OR Linux-OpenIB
/*
 * Copyright (c) 2022, Microsoft Corporation. All rights reserved.
 */

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdatomic.h>
#include <util/compiler.h>
#include <util/util.h>
#include <sys/mman.h>

#include <infiniband/driver.h>

#include <infiniband/kern-abi.h>
#include <rdma/mana-abi.h>
#include <kernel-abi/mana-abi.h>

#include "mana.h"

DECLARE_DRV_CMD(mana_create_qp, IB_USER_VERBS_CMD_CREATE_QP, mana_ib_create_qp,
		mana_ib_create_qp_resp);

DECLARE_DRV_CMD(mana_create_qp_ex, IB_USER_VERBS_EX_CMD_CREATE_QP,
		mana_ib_create_qp_rss, mana_ib_create_qp_rss_resp);

DECLARE_DRV_CMD(mana_create_rc_qp, IB_USER_VERBS_CMD_CREATE_QP,
		mana_ib_create_rc_qp, mana_ib_create_rc_qp_resp);

static struct ibv_qp *mana_create_qp_raw(struct ibv_pd *ibpd,
					 struct ibv_qp_init_attr *attr)
{
	int ret;
	struct mana_cq *cq;
	struct mana_qp *qp;
	struct mana_pd *pd = container_of(ibpd, struct mana_pd, ibv_pd);
	struct mana_parent_domain *mpd;
	uint32_t port;

	struct mana_create_qp qp_cmd = {};
	struct mana_create_qp_resp qp_resp = {};
	struct mana_ib_create_qp *qp_cmd_drv;
	struct mana_ib_create_qp_resp *qp_resp_drv;

	struct mana_context *ctx = to_mctx(ibpd->context);

	/* This is a RAW QP, pd is a parent domain with port number */
	if (!pd->mprotection_domain) {
		verbs_err(verbs_get_ctx(ibpd->context),
			  "Create RAW QP should use parent domain\n");
		errno = EINVAL;
		return NULL;
	}

	mpd = container_of(pd, struct mana_parent_domain, mpd);
	port = (uint32_t)(uintptr_t)mpd->pd_context;

	cq = to_mana_cq(attr->send_cq);

	if (!ctx->extern_alloc.alloc || !ctx->extern_alloc.free) {
		verbs_err(verbs_get_ctx(ibpd->context),
			  "RAW QP requires extern alloc for buffers\n");
		errno = EINVAL;
		return NULL;
	}

	qp = calloc(1, sizeof(*qp));
	if (!qp)
		return NULL;

	qp->raw_qp.send_buf_size =
		attr->cap.max_send_wr * get_wqe_size(attr->cap.max_send_sge);
	qp->raw_qp.send_buf_size = align_hw_size(qp->raw_qp.send_buf_size);

	qp->raw_qp.send_buf = ctx->extern_alloc.alloc(qp->raw_qp.send_buf_size,
					       ctx->extern_alloc.data);
	if (!qp->raw_qp.send_buf) {
		errno = ENOMEM;
		goto free_qp;
	}

	qp_cmd_drv = &qp_cmd.drv_payload;
	qp_resp_drv = &qp_resp.drv_payload;

	qp_cmd_drv->sq_buf_addr = (uintptr_t)qp->raw_qp.send_buf;
	qp_cmd_drv->sq_buf_size = qp->raw_qp.send_buf_size;
	qp_cmd_drv->port = port;

	ret = ibv_cmd_create_qp(ibpd, &qp->ibqp.qp, attr, &qp_cmd.ibv_cmd,
				sizeof(qp_cmd), &qp_resp.ibv_resp,
				sizeof(qp_resp));
	if (ret) {
		verbs_err(verbs_get_ctx(ibpd->context), "Create QP failed\n");
		ctx->extern_alloc.free(qp->raw_qp.send_buf, ctx->extern_alloc.data);
		errno = ret;
		goto free_qp;
	}

	qp->raw_qp.sqid = qp_resp_drv->sqid;
	qp->raw_qp.tx_vp_offset = qp_resp_drv->tx_vp_offset;
	qp->raw_qp.send_wqe_count = attr->cap.max_send_wr;

	cq->cqid = qp_resp_drv->cqid;

	return &qp->ibqp.qp;

free_qp:
	free(qp);
	return NULL;
}

static int create_shadow_queue(struct shadow_queue *queue, uint32_t max_wr, uint32_t wqe_size)
{
	queue->length = align_next_power2(max_wr);
	queue->index_mask = align_next_power2(max_wr + 1) - 1;
	queue->stride = align(wqe_size, sizeof(uint64_t));
	queue->size = queue->stride * align_next_power2(max_wr + 1);
	queue->buffer = mmap(NULL, queue->size, PROT_READ | PROT_WRITE,
			MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);

	if (queue->buffer == MAP_FAILED) {
		queue->buffer = NULL;
		return -1;
	}

	return 0;
}

static struct mana_ib_rollback_shared_mem
	*mana_ib_get_rollback_sh_mem(struct mana_qp *qp)
{
	struct mana_ib_rollback_shared_mem *rb_shmem;
	struct mana_gdma_queue *req_sq =
		&qp->rc_qp.queues[RC_SEND_QUEUE_REQUESTER];

	rb_shmem = (struct mana_ib_rollback_shared_mem*)
		((uint8_t*)req_sq->buffer + req_sq->size -
		sizeof (struct mana_ib_rollback_shared_mem));

	assert(BYTE_OFFSET(rb_shmem) == 0); //page - aligned

	return rb_shmem;
}

static void mana_ib_init_rb_shmem(struct mana_qp *qp)
{
	struct mana_ib_rollback_shared_mem *rb_shmem =
		mana_ib_get_rollback_sh_mem(qp);

	rb_shmem->signature = RNIC_ROLLBACK_SHARED_MEM_SIG;
	rb_shmem->size = sizeof(struct mana_ib_rollback_shared_mem);
}

static void mana_ib_reset_rb_shmem(struct mana_qp *qp)
{
	struct mana_ib_rollback_shared_mem *rb_shmem =
		mana_ib_get_rollback_sh_mem(qp);
	rb_shmem->left_offset = 0;
	rb_shmem->right_offset = 0;
}

static struct ibv_qp *mana_create_qp_rc(struct ibv_pd *ibpd,
					struct ibv_qp_init_attr *attr)
{
	int ret, i;
	struct mana_qp *qp;
	struct mana_context *ctx = to_mctx(ibpd->context);
	struct mana_create_rc_qp qp_cmd = {};
	struct mana_create_rc_qp_resp qp_resp = {};
	struct mana_ib_create_rc_qp *qp_cmd_drv;
	struct mana_ib_create_rc_qp_resp *qp_resp_drv;
	uint32_t queue_size;

	qp = calloc(1, sizeof(*qp));
	if (!qp)
		return NULL;

	if (create_shadow_queue(&qp->shadow_sq,attr->cap.max_send_wr,sizeof(struct rc_sq_shadow_wqe))) {
		verbs_err(verbs_get_ctx(ibpd->context), "Failed to alloc sq shadow queue\n");
		goto cleanup;
	}

	if (create_shadow_queue(&qp->shadow_rq,attr->cap.max_send_wr,sizeof(struct rc_rq_shadow_wqe))) {
		verbs_err(verbs_get_ctx(ibpd->context), "Failed to alloc rc shadow queue\n");
		goto cleanup;
	}

	/* Remove the rollback shared mem offset to maintain power of 2 for q size */
	qp->rc_qp.queues[RC_RECV_QUEUE_REQUESTER].wq_size =
		align_hw_size(MANA_IB_MAX_RD_ATOMIC * MAX_RX_WQE_SIZE);
	qp->rc_qp.queues[RC_RECV_QUEUE_RESPONDER].wq_size =
		align_hw_size(attr->cap.max_recv_wr * get_wqe_size(attr->cap.max_recv_sge));
	qp->rc_qp.queues[RC_SEND_QUEUE_REQUESTER].wq_size =
		align_hw_size(attr->cap.max_send_wr * get_large_wqe_size(attr->cap.max_send_sge));
	qp->rc_qp.queues[RC_SEND_QUEUE_RESPONDER].wq_size =
		align_hw_size(MANA_IB_MAX_DEST_RD_ATOMIC * MANA_IB_RESPONSE_WQE_SIZE * 2);

	for (i = 0; i < RC_QUEUE_TYPE_MAX; i++) {
		queue_size = qp->rc_qp.queues[i].wq_size;
		if (i == RC_SEND_QUEUE_RESPONDER)
			queue_size += sizeof(struct mana_ib_rollback_shared_mem);

		qp->rc_qp.queues[i].size = queue_size;
		qp->rc_qp.queues[i].buffer = mmap(NULL, queue_size, PROT_READ | PROT_WRITE,
			MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);

		if (qp->rc_qp.queues[i].buffer == MAP_FAILED) {
			qp->rc_qp.queues[i].buffer = NULL;
			verbs_err(verbs_get_ctx(ibpd->context), "Failed to allocate memory");
			errno = ENOMEM;
			goto cleanup;
		}
	}

	mana_ib_init_rb_shmem(qp);

	qp_cmd_drv = &qp_cmd.drv_payload;
	qp_resp_drv = &qp_resp.drv_payload;

	for (i = 0; i < RC_QUEUE_TYPE_MAX; ++i) {
		qp_cmd_drv->queue_addrs[i] = (uintptr_t)qp->rc_qp.queues[i].buffer;
		qp_cmd_drv->queue_sizes[i] = qp->rc_qp.queues[i].size;
	}

	ret = ibv_cmd_create_qp(ibpd, &qp->ibqp.qp, attr, &qp_cmd.ibv_cmd,
				sizeof(qp_cmd), &qp_resp.ibv_resp,
				sizeof(qp_resp));
	if (ret) {
		verbs_err(verbs_get_ctx(ibpd->context), "Create QP failed\n");
		errno = ret;
		goto cleanup;
	}

	list_add(&to_mana_cq(attr->send_cq)->send_qp_list, &qp->send_cq_node);
	list_add(&to_mana_cq(attr->recv_cq)->recv_qp_list, &qp->recv_cq_node);

	for (i = 0; i < RC_QUEUE_TYPE_MAX; ++i) {
		qp->rc_qp.queues[i].id = qp_resp_drv->queue_ids[i];
		ctx->lookup_table[qp->rc_qp.queues[i].id] = qp;
	}

	qp->ibqp.qp.qp_num = qp->rc_qp.queues[RC_RECV_QUEUE_RESPONDER].id;
	return &qp->ibqp.qp;

cleanup:
	if (qp->shadow_sq.buffer)
		munmap(qp->shadow_sq.buffer, qp->shadow_sq.size);

	if (qp->shadow_rq.buffer)
		munmap(qp->shadow_rq.buffer, qp->shadow_rq.size);

	for (i = 0; i < RC_QUEUE_TYPE_MAX; ++i)
		if (qp->rc_qp.queues[i].buffer)
			munmap(qp->rc_qp.queues[i].buffer, qp->rc_qp.queues[i].size);

	if (qp)
		free(qp);

	return NULL;
}

struct ibv_qp *mana_create_qp(struct ibv_pd *ibpd,
			      struct ibv_qp_init_attr *attr)
{
	switch (attr->qp_type) {
	case IBV_QPT_RAW_PACKET:
		return mana_create_qp_raw(ibpd, attr);
	case IBV_QPT_RC:
		return mana_create_qp_rc(ibpd, attr);
	default:
		verbs_err(verbs_get_ctx(ibpd->context),
			  "QP type %u is not supported\n", attr->qp_type);
		errno = EINVAL;
	}

	return NULL;
}

static inline void reset_shadow_queue(struct shadow_queue *queue)
{
	queue->cons_idx = 0;
	queue->prod_idx = 0;
	queue->next_to_complete_idx = 0;
}

static void mana_ib_modify_rc_qp(struct mana_qp *qp, struct ibv_qp_attr *attr, int attr_mask)
{
	int i;
	switch (attr->qp_state) {
		case IBV_QPS_RESET:
			SWITCH_FALLTHROUGH;
		case IBV_QPS_INIT:
			for (i = 0; i < RC_QUEUE_TYPE_MAX; ++i) {
				qp->rc_qp.queues[i].head = 0;
				qp->rc_qp.queues[i].tail = 0;
			}
			reset_shadow_queue(&qp->shadow_rq);
			reset_shadow_queue(&qp->shadow_sq);
			mana_ib_reset_rb_shmem(qp);
			break;
		case IBV_QPS_RTR:
			qp->rc_qp.rq_highest_completed_psn = decrement_psn(attr->rq_psn);
			break;
		case IBV_QPS_RTS:
			qp->rc_qp.ssn = 1;
			qp->rc_qp.sq_psn = attr->sq_psn;
			qp->rc_qp.sq_highest_completed_psn = decrement_psn(attr->sq_psn);
			qp->rc_qp.sq_last_armed_psn = decrement_psn(attr->sq_psn);
			mana_ib_reset_rb_shmem(qp);
			break;
		default:
			break;
	}
}

int mana_modify_qp(struct ibv_qp *ibqp, struct ibv_qp_attr *attr, int attr_mask)
{
	struct mana_qp *qp = to_mana_qp(ibqp);
	struct ibv_modify_qp cmd = {};
	int err;

	if (ibqp->qp_type != IBV_QPT_RC)
		return EOPNOTSUPP;

	if (!(attr_mask & IBV_QP_STATE))
		return 0;

	err = ibv_cmd_modify_qp(ibqp, attr, attr_mask, &cmd, sizeof(cmd));
	if (err) {
		verbs_err(verbs_get_ctx(ibqp->context), "Failed to modify qp\n");
		return err;
	}

	mana_ib_modify_rc_qp(qp, attr, attr_mask);
	return 0;
}

int mana_destroy_qp(struct ibv_qp *ibqp)
{
	int ret, i;
	struct mana_qp *qp = container_of(ibqp, struct mana_qp, ibqp.qp);
	struct mana_context *ctx = to_mctx(ibqp->context);

	ret = ibv_cmd_destroy_qp(ibqp);
	if (ret) {
		verbs_err(verbs_get_ctx(ibqp->context), "Destroy QP failed\n");
		return ret;
	}

	if (ibqp->qp_type == IBV_QPT_RAW_PACKET) {
		ctx->extern_alloc.free(qp->raw_qp.send_buf, ctx->extern_alloc.data);
	}

	if (ibqp->qp_type == IBV_QPT_RC) {
		if (qp->shadow_sq.buffer)
			munmap(qp->shadow_sq.buffer, qp->shadow_sq.size);

		if (qp->shadow_rq.buffer)
			munmap(qp->shadow_rq.buffer, qp->shadow_rq.size);

		for (i = 0; i < RC_QUEUE_TYPE_MAX; ++i)
			if (qp->rc_qp.queues[i].buffer)
				munmap(qp->rc_qp.queues[i].buffer, qp->rc_qp.queues[i].size);
	}

	free(qp);

	return 0;
}

static struct ibv_qp *mana_create_qp_ex_raw(struct ibv_context *context,
					    struct ibv_qp_init_attr_ex *attr)
{
	struct mana_create_qp_ex cmd = {};
	struct mana_ib_create_qp_rss *cmd_drv;
	struct mana_create_qp_ex_resp resp = {};
	struct mana_ib_create_qp_rss_resp *cmd_resp;
	struct mana_qp *qp;
	struct mana_pd *pd = container_of(attr->pd, struct mana_pd, ibv_pd);
	struct mana_parent_domain *mpd;
	uint32_t port;
	int ret;

	cmd_drv = &cmd.drv_payload;
	cmd_resp = &resp.drv_payload;

	/* For a RAW QP, pd is a parent domain with port number */
	if (!pd->mprotection_domain) {
		verbs_err(verbs_get_ctx(context),
			  "RAW QP needs to be on a parent domain\n");
		errno = EINVAL;
		return NULL;
	}

	if (attr->rx_hash_conf.rx_hash_key_len !=
	    MANA_IB_TOEPLITZ_HASH_KEY_SIZE_IN_BYTES) {
		verbs_err(verbs_get_ctx(context),
			  "Invalid RX hash key length\n");
		errno = EINVAL;
		return NULL;
	}

	mpd = container_of(pd, struct mana_parent_domain, mpd);
	port = (uint32_t)(uintptr_t)mpd->pd_context;

	qp = calloc(1, sizeof(*qp));
	if (!qp)
		return NULL;

	cmd_drv->rx_hash_fields_mask = attr->rx_hash_conf.rx_hash_fields_mask;
	cmd_drv->rx_hash_function = attr->rx_hash_conf.rx_hash_function;
	cmd_drv->rx_hash_key_len = attr->rx_hash_conf.rx_hash_key_len;
	if (cmd_drv->rx_hash_key_len)
		memcpy(cmd_drv->rx_hash_key, attr->rx_hash_conf.rx_hash_key,
		       cmd_drv->rx_hash_key_len);

	cmd_drv->port = port;

	ret = ibv_cmd_create_qp_ex2(context, &qp->ibqp, attr, &cmd.ibv_cmd,
				    sizeof(cmd), &resp.ibv_resp, sizeof(resp));
	if (ret) {
		verbs_err(verbs_get_ctx(context), "Create QP EX failed\n");
		free(qp);
		errno = ret;
		return NULL;
	}

	if (attr->rwq_ind_tbl) {
		struct mana_rwq_ind_table *ind_table =
			container_of(attr->rwq_ind_tbl,
				     struct mana_rwq_ind_table, ib_ind_table);
		for (int i = 0; i < ind_table->ind_tbl_size; i++) {
			struct mana_wq *wq = container_of(ind_table->ind_tbl[i],
							  struct mana_wq, ibwq);
			struct mana_cq *cq = to_mana_cq(wq->ibwq.cq);
			wq->wqid = cmd_resp->entries[i].wqid;
			cq->cqid = cmd_resp->entries[i].cqid;
		}
	}

	return &qp->ibqp.qp;
}

struct ibv_qp *mana_create_qp_ex(struct ibv_context *context,
				 struct ibv_qp_init_attr_ex *attr)
{
	switch (attr->qp_type) {
	case IBV_QPT_RAW_PACKET:
		return mana_create_qp_ex_raw(context, attr);
	default:
		verbs_err(verbs_get_ctx(context),
			  "QP type %u is not supported\n", attr->qp_type);
		errno = EINVAL;
	}

	return NULL;
}
