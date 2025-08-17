/*
 * Copyright (C) Ovyl
 */

/**
 * @file gpio_emul_net_protocol_handler.c
 * @author Evan Stoddard
 * @brief
 */

#include "gpio_emul_net_protocol_handler.h"

#include <errno.h>
#include <stddef.h>
#include <unistd.h>

/*****************************************************************************
 * Definitions
 *****************************************************************************/

/*****************************************************************************
 * Structs, Unions, Enums, & Typedefs
 *****************************************************************************/

typedef int (*gpio_emul_net_handler_t)(
    int fd, const gpio_emul_net_proto_header_t *header);

/*****************************************************************************
 * Handler Prototypes
 *****************************************************************************/

/**
 * @brief [TODO:description]
 *
 * @param fd [TODO:parameter]
 * @param header [TODO:parameter]
 * @return [TODO:return]
 */
static int prv_handle_ident_msg(int fd,
                                const gpio_emul_net_proto_header_t *header);

/**
 * @brief [TODO:description]
 *
 * @param fd [TODO:parameter]
 * @param header [TODO:parameter]
 * @return [TODO:return]
 */
static int prv_handle_pin_flags_msg(int fd,
                                    const gpio_emul_net_proto_header_t *header);

/**
 * @brief [TODO:description]
 *
 * @param fd [TODO:parameter]
 * @param header [TODO:parameter]
 * @return [TODO:return]
 */
static int
prv_handle_pin_values_msg(int fd, const gpio_emul_net_proto_header_t *header);

/*****************************************************************************
 * Variables
 *****************************************************************************/

/**
 * @brief [TODO:description]
 */
static struct {
  gpio_emul_net_write_func_t write_func;
  void *write_ctx;
  gpio_emul_net_callbacks_t callbacks;
} prv_inst;

/**
 * @brief [TODO:description]
 */
static gpio_emul_net_handler_t prv_handlers[GPIO_EMUL_PROTO_NUM_IDS] = {
    [GPIO_EMUL_PROTO_MSG_IDENT] = prv_handle_ident_msg,
    [GPIO_EMUL_PROTO_MSG_PIN_FLAGS] = prv_handle_pin_flags_msg,
    [GPIO_EMUL_PROTO_MSG_PIN_VALUES] = prv_handle_pin_values_msg,
};

/*****************************************************************************
 * Private Functions
 *****************************************************************************/

/**
 * @brief Wrapper for write function
 *
 * @param data Pointer to data
 * @param size Size of buffer to write
 * @return Returns 0 on success
 */
static inline int prv_write(const void *data, size_t size) {
  if (prv_inst.write_func == NULL) {
    return -EIO;
  }

  return prv_inst.write_func(data, size, prv_inst.write_ctx);
}

/*****************************************************************************
 * Functions
 *****************************************************************************/

int gpio_emul_net_protocol_data_available(int fd) {
  gpio_emul_net_proto_header_t header = {0};

  ssize_t bytes_read = read(fd, &header, sizeof(header));

  if (bytes_read < 0) {
    return bytes_read;
  }

  if (bytes_read == 0) {
    return -ECONNABORTED;
  }

  if (bytes_read != sizeof(header)) {
    return -EBADMSG;
  }

  if (header.magic != GPIO_EMUL_NET_PROTO_HEADER_MAGIC) {
    return -EBADMSG;
  }

  if (header.message_type >= GPIO_EMUL_PROTO_NUM_IDS) {
    return -EBADMSG;
  }

  if (prv_handlers[header.message_type] == NULL) {
    return -ENOENT;
  }

  return prv_handlers[header.message_type](fd, &header);
}

void gpio_emul_net_set_write_function(gpio_emul_net_write_func_t func,
                                      void *ctx) {
  prv_inst.write_func = func;
  prv_inst.write_ctx = ctx;
}

gpio_emul_net_callbacks_t *gpio_emul_net_callbacks(void) {
  return &prv_inst.callbacks;
}

int gpio_emul_net_write_ident_message(uint32_t num_gpios) {
  gpio_emul_net_ident_t msg = {0};
  gpio_emul_net_ident_init(&msg);

  msg.payload.num_gpios = num_gpios;

  return prv_write(&msg, sizeof(msg));
}

int gpio_net_emul_write_gpio_flags_message(uint32_t pin_number,
                                           uint32_t flags) {
  gpio_emul_net_pin_flags_t msg = {0};
  gpio_emul_net_pin_flags_init(&msg);

  msg.payload.pin_number = pin_number;
  msg.payload.flags = flags;
  return prv_write(&msg, sizeof(msg));
}

int gpio_emul_net_write_pin_values_message(uint32_t pin_mask, uint32_t values) {
  gpio_emul_net_proto_pin_values_t msg = {0};
  gpio_emul_net_proto_pin_values_init(&msg);

  msg.payload.pin_mask = pin_mask;
  msg.payload.pin_values = values;

  return prv_write(&msg, sizeof(msg));
}
/*****************************************************************************
 * Protocol Handler
 *****************************************************************************/

static int prv_handle_ident_msg(int fd,
                                const gpio_emul_net_proto_header_t *header) {
  gpio_emul_net_ident_payload_t payload = {0};

  if (header->payload_len_bytes != sizeof(payload)) {
    return -EMSGSIZE;
  }

  ssize_t ret = read(fd, &payload, sizeof(payload));

  if (ret < 0) {
    return ret;
  }

  if (ret == 0) {
    return -ECONNABORTED;
  }

  if (ret != header->payload_len_bytes) {
    return -EBADMSG;
  }

  // TODO: Handle callback
  if (prv_inst.callbacks.on_ident != NULL) {
    prv_inst.callbacks.on_ident(&payload);
  }

  return 0;
}

static int
prv_handle_pin_flags_msg(int fd, const gpio_emul_net_proto_header_t *header) {
  gpio_emul_net_pin_flags_payload_t payload = {0};

  if (header->payload_len_bytes != sizeof(payload)) {
    return -EMSGSIZE;
  }

  ssize_t ret = read(fd, &payload, sizeof(payload));

  if (ret < 0) {
    return ret;
  }

  if (ret == 0) {
    return -ECONNABORTED;
  }

  if (ret != header->payload_len_bytes) {
    return -EBADMSG;
  }

  // TODO: Handle callback
  if (prv_inst.callbacks.on_pin_flags != NULL) {
    prv_inst.callbacks.on_pin_flags(&payload);
  }

  return 0;
}

static int
prv_handle_pin_values_msg(int fd, const gpio_emul_net_proto_header_t *header) {
  gpio_emul_net_proto_pin_values_payload_t payload = {0};

  if (header->payload_len_bytes != sizeof(payload)) {
    return -EMSGSIZE;
  }

  ssize_t ret = read(fd, &payload, sizeof(payload));

  if (ret < 0) {
    return ret;
  }

  if (ret == 0) {
    return -ECONNABORTED;
  }

  if (ret != header->payload_len_bytes) {
    return -EBADMSG;
  }

  // TODO: Handle callback
  if (prv_inst.callbacks.on_pin_values != NULL) {
    prv_inst.callbacks.on_pin_values(&payload);
  }

  return 0;
}
