/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef LWM2M_RW_SENML_JSON_H_
#define LWM2M_RW_SENML_JSON_H_

#include "lwm2m_object.h"

extern const struct lwm2m_writer senml_json_writer;
extern const struct lwm2m_reader senml_json_reader;

/* Init Context format for coap blocking */
void lwm2m_senml_json_context_init(struct lwm2m_senml_json_context *ctx);
/* General Read single Path operation */
int do_read_op_senml_json(struct lwm2m_message *msg);
/* General Write single Path operation */
int do_write_op_senml_json(struct lwm2m_message *msg);

/* Send opearation builder */
int do_send_op_senml_json(struct lwm2m_message *msg, sys_slist_t *lwm_path_list);
/* API for call composite READ from engine */
int do_composite_read_op_senml_json(struct lwm2m_message *msg);

#endif /* LWM2M_RW_SENML_JSON_H_ */
