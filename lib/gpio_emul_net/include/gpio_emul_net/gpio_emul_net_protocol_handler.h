/*
 * Copyright (C) Ovyl
 */

/**
 * @file gpio_emul_net_protocol_handler.h
 * @author Evan Stoddard
 * @brief
 */

#ifndef gpio_emul_net_protocol_handler_h
#define gpio_emul_net_protocol_handler_h

#include <stddef.h>
#include <stdint.h>

#include "gpio_emul_net_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/*****************************************************************************
 * Definitions
 *****************************************************************************/

/*****************************************************************************
 * Structs, Unions, Enums, & Typedefs
 *****************************************************************************/

typedef int (*gpio_emul_net_write_func_t)(const void *data, size_t size);

typedef void (*gpio_emul_net_on_ident_t)(
    gpio_emul_net_ident_payload_t *payload);

/**
 * @typedef gpio_emul_net_callbacks_t
 * @brief [TODO:description]
 *
 */
typedef struct gpio_emul_net_callbacks_t {
  gpio_emul_net_on_ident_t on_ident;
} gpio_emul_net_callbacks_t;

/*****************************************************************************
 * Function Prototypes
 *****************************************************************************/

/**
 * @brief Function called when the socket has data available
 *
 * @param fd File Descriptor of socket
 * @return Returns 0 if message is successfully received and processed
 */
int gpio_emul_net_protocol_data_available(int fd);

/**
 * @brief Set write function
 *
 * @param func Pointer to write function
 */
void gpio_emul_net_set_write_function(gpio_emul_net_write_func_t func);

gpio_emul_net_callbacks_t *gpio_emul_net_callbacks(void);

/**
 * @brief Write identification message
 *
 * @param num_gpios Number of GPIOs on controller
 * @return Returns 0 on success
 */
int gpio_emul_net_write_ident_message(uint32_t num_gpios);

#ifdef __cplusplus
}
#endif
#endif /* gpio_emul_net_protocol_handler_h */
