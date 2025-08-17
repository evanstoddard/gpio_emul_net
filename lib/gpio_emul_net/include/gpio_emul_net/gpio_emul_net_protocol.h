/*
 * Copyright (C) Ovyl
 */

/**
 * @file gpio_emul_net_protocol.h
 * @author Evan Stoddard
 * @brief
 */

#ifndef gpio_emul_net_protocol_h
#define gpio_emul_net_protocol_h

#include <stdint.h>

#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/*****************************************************************************
 * Definitions
 *****************************************************************************/

#define GPIO_EMUL_NET_PROTO_HEADER_MAGIC (0x6910CAFE)

#define GPIO_EMUL_NET_PROTO_HEADER_RESERVED_WORDS (4U)

#define GPIO_EMUL_MSG_DEFINE(msg_name, msg_id, payload_definition)             \
  typedef struct __attribute__((__packed__)) msg_name##_payload_t              \
      payload_definition msg_name##_payload_t;                                 \
                                                                               \
  typedef struct __attribute__((__packed__)) msg_name##_t {                    \
    gpio_emul_net_proto_header_t header;                                       \
    msg_name##_payload_t payload;                                              \
  } msg_name##_t;                                                              \
                                                                               \
  static void __attribute__((__unused__)) msg_name##_init(msg_name##_t *msg) { \
    memset(msg, 0, sizeof(*msg));                                              \
    msg->header.magic = GPIO_EMUL_NET_PROTO_HEADER_MAGIC;                      \
    msg->header.message_type = msg_id;                                         \
    msg->header.payload_len_bytes = sizeof(msg_name##_payload_t);              \
  }

/*****************************************************************************
 * Structs, Unions, Enums, & Typedefs
 *****************************************************************************/

/**
 * @typedef gpio_emul_net_proto_header_t
 * @brief Protocol header
 *
 */
typedef struct __attribute__((__packed__)) gpio_emul_net_proto_header_t {
  /**
   * @brief Magic value to denote this is a GPIO emul protocol message
   */
  uint32_t magic;

  /**
   * @brief Flags for this message (currently unused and ignored)
   */
  uint32_t flags;

  /**
   * @brief Message type
   */
  uint32_t message_type;

  /**
   * @brief Unused reserved bytes to allow the header to grow
   */
  uint32_t reserved[GPIO_EMUL_NET_PROTO_HEADER_RESERVED_WORDS];

  /**
   * @brief Length of accompanying payload in bytes
   */
  uint32_t payload_len_bytes;
} gpio_emul_net_proto_header_t;

/**
 * @brief Message IDs
 */
typedef enum {
  GPIO_EMUL_PROTO_MSG_IDENT,
  GPIO_EMUL_PROTO_NUM_IDS,
} gpio_emul_proto_msg_type_t;

GPIO_EMUL_MSG_DEFINE(gpio_emul_net_ident, GPIO_EMUL_PROTO_MSG_IDENT,
                     { uint32_t num_gpios; });

#ifdef __cplusplus
}
#endif
#endif /* gpio_emul_net_protocol_h */
