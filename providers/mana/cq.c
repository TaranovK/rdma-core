// SPDX-License-Identifier: GPL-2.0 OR Linux-OpenIB
/*
 * Copyright (c) 2023, Microsoft Corporation. All rights reserved.
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

DECLARE_DRV_CMD(mana_create_cq, IB_USER_VERBS_CMD_CREATE_CQ, mana_ib_create_cq,
		empty);

DECLARE_DRV_CMD(mana_create_cq_ex, IB_USER_VERBS_EX_CMD_CREATE_CQ, mana_ib_create_cq_ex,
		mana_ib_create_cq_ex_resp);

static int mana_cmd_create_cq(struct ibv_context *context,
			      struct ibv_cq_init_attr_ex *cq_attr,
			      struct mana_cq *cq)
{
	struct mana_create_cq cmd = {};
	struct mana_create_cq_resp resp = {};
	struct mana_ib_create_cq *cmd_drv;

	cmd_drv = &cmd.drv_payload;
	cmd_drv->buf_addr = (uintptr_t)cq->buf;

	return ibv_cmd_create_cq(context, cq_attr->cqe, cq_attr->channel, cq_attr->comp_vector,
				 &cq->ibcq.cq, &cmd.ibv_cmd, sizeof(cmd),
				 &resp.ibv_resp, sizeof(resp));
}

static int mana_cmd_create_cq_ex(struct ibv_context *context,
				 struct ibv_cq_init_attr_ex *cq_attr,
				 struct mana_cq *cq)
{
	struct mana_create_cq_ex cmd = {};
	struct mana_create_cq_ex_resp resp = {};
	struct mana_ib_create_cq_ex *cmd_drv;
	int ret;

	cmd_drv = &cmd.drv_payload;
	cmd_drv->buf_addr = (uintptr_t)cq->buf;
	cmd_drv->buf_size = cq->buf_size;

	ret = ibv_cmd_create_cq_ex(context, cq_attr, &cq->ibcq, &cmd.ibv_cmd,
				   sizeof(cmd), &resp.ibv_resp, sizeof(resp), 0);
	if (!ret)
		cq->cqid = resp.cqid;

	return ret;
}

static struct ibv_cq_ex *create_cq(struct ibv_context *context,
				   struct ibv_cq_init_attr_ex *cq_attr)
{
	struct mana_context *ctx = to_mctx(context);
	struct mana_cq *cq;
	int cq_size;
	int ret;
	bool bnic_cq = true;
	bool external_alloc = (ctx->extern_alloc.alloc && ctx->extern_alloc.free);

	if (!external_alloc || (ctx->flags & MANA_CTX_FLAG_FORCE_RNIC))
		bnic_cq = false;

	cq = calloc(1, sizeof(*cq));
	if (!cq)
		return NULL;

	cq->cqe = cq_attr->cqe;
	cq_size = cq->cqe * COMP_ENTRY_SIZE;
	cq_size = roundup_pow_of_two(cq_size);
	cq_size = align(cq_size, MANA_PAGE_SIZE);
	cq->buf_size = cq_size;

	if (external_alloc) {
		cq->buf = ctx->extern_alloc.alloc(cq->buf_size, ctx->extern_alloc.data);
		if (!cq->buf) {
			errno = ENOMEM;
			goto free_cq;
		}
	} else {
		cq->buf = mmap(NULL, cq->buf_size, PROT_READ | PROT_WRITE,
			       MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
		if (cq->buf == MAP_FAILED) {
			errno = ENOMEM;
			goto free_cq;
		}
	}

	if (bnic_cq)
		ret = mana_cmd_create_cq(context, cq_attr, cq);
	else
		ret = mana_cmd_create_cq_ex(context, cq_attr, cq);

	if (ret) {
		verbs_err(verbs_get_ctx(context), "Failed to Create CQ\n");
		if (external_alloc)
			ctx->extern_alloc.free(cq->buf, ctx->extern_alloc.data);
		else
			munmap(cq->buf, cq->buf_size);
		errno = ret;
		goto free_cq;
	}

	return &cq->ibcq.cq_ex;

free_cq:
	free(cq);
	return NULL;
}

struct ibv_cq *mana_create_cq(struct ibv_context *context, int cqe,
			      struct ibv_comp_channel *channel, int comp_vector)
{
	struct ibv_cq_init_attr_ex attr_ex = {
		.cqe = cqe,
		.channel = channel,
		.comp_vector = comp_vector
	};
	struct ibv_cq_ex *cq_ex = create_cq(context, &attr_ex);

	return cq_ex ? ibv_cq_ex_to_cq(cq_ex) : NULL;
}

int mana_destroy_cq(struct ibv_cq *ibcq)
{
	int ret;
	struct mana_cq *cq = container_of(ibcq, struct mana_cq, ibcq.cq);
	struct mana_context *ctx = to_mctx(ibcq->context);

	ret = ibv_cmd_destroy_cq(ibcq);
	if (ret) {
		verbs_err(verbs_get_ctx(ibcq->context),
			  "Failed to Destroy CQ\n");
		return ret;
	}

	if (ctx->extern_alloc.free)
		ctx->extern_alloc.free(cq->buf, ctx->extern_alloc.data);
	else
		munmap(cq->buf, cq->buf_size);

	free(cq);
	return ret;
}

int mana_poll_cq(struct ibv_cq *ibcq, int nwc, struct ibv_wc *wc)
{
	/* This version of driver supports RAW QP only.
	 * Polling CQ is done directly in the application.
	 */
	return EOPNOTSUPP;
}
