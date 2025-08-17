/*
 * Copyright (C) Evan Stoddard
 */

/**
 * @file gpio_emul_net.c
 * @author Evan Stoddard
 * @brief
 */

#define DT_DRV_COMPAT zephyr_gpio_emul_net

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_emul.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <gpio_emul_net/gpio_emul_net_protocol.h>

/*****************************************************************************
 * Definitions
 *****************************************************************************/

LOG_MODULE_REGISTER(gpio_emul_net, CONFIG_GPIO_LOG_LEVEL);

#define GPIO_EMUL_DATA(ptr) ((struct gpio_emul_net_data *)ptr)

#define GPIO_EMUL_NET_THREAD_PRIORITY (1U)

#define GPIO_EMUL_NET_THREAD_STACK_SIZE_BYTES (2048U)

#define GPIO_EMUL_NET_CONNECT_RETRY_DELAY_S (5U)

/*****************************************************************************
 * Typedefs, Structs, & Enums
 *****************************************************************************/

struct gpio_emul_net_data {
  struct k_thread thread;
  k_tid_t thread_id;

  void *stack;
  size_t stack_size;

  int server_fd;
  struct sockaddr_un server_addr;

  const struct device *parent;
};

struct gpio_emul_net_config {
  struct gpio_emul_net_data *data;
};

/*****************************************************************************
 * Variables
 *****************************************************************************/

static const char *prv_default_path = "/tmp/gpio_emu_sock";

/*****************************************************************************
 * Private Functions
 *****************************************************************************/

/**
 * @brief [TODO:description]
 *
 * @param dev [TODO:parameter]
 * @return [TODO:return]
 */
static int prv_setup_server_socket(const struct device *dev) {
  struct gpio_emul_net_data *data = GPIO_EMUL_DATA(dev->data);

  data->server_fd = socket(AF_UNIX, SOCK_STREAM, 0);

  if (data->server_fd < 0) {
    int err = errno;

    LOG_ERR("Failed to create socket for connection to GPIO server: %d", err);
    return -EIO;
  }

  return 0;
}

/**
 * @brief [TODO:description]
 *
 * @param arg1 [TODO:parameter]
 * @param arg2 [TODO:parameter]
 * @param arg3 [TODO:parameter]
 */
static void prv_thread_handler(void *arg1, void *arg2, void *arg3) {
  const struct device *dev = (const struct device *)arg1;
  struct gpio_emul_net_data *data = GPIO_EMUL_DATA(dev->data);

  while (true) {
    int ret = prv_setup_server_socket(dev);

    if (ret < 0) {
      return;
    }

    ret = connect(data->server_fd, (const struct sockaddr *)&data->server_addr,
                  sizeof(data->server_addr));

    if (ret < 0) {
      int err = errno;
      LOG_ERR("Failed to connect to server: %d", err);

      close(data->server_fd);

      // Try again after delay
      LOG_ERR("Trying to connect to server in %u seconds.",
              GPIO_EMUL_NET_CONNECT_RETRY_DELAY_S);

      k_sleep(K_SECONDS(GPIO_EMUL_NET_CONNECT_RETRY_DELAY_S));

      continue;
    } else {
      LOG_INF("Successfully connected to GPIO server socket.");

      gpio_emul_net_ident_t msg = {0};
      msg.header.payload_len_bytes = sizeof(gpio_emul_net_ident_payload_t);
      msg.payload.num_gpios = 69;

      send(data->server_fd, &msg, sizeof(msg), 0);
    }

    k_sleep(K_FOREVER);
  }
}

static inline uint32_t prv_num_gpios_from_bitmask(uint32_t bitmask) {
  if (bitmask == 0) {
    return 0;
  }
  return 32 - __builtin_clz(bitmask);
}

/*****************************************************************************
 * Driver Bindings
 *****************************************************************************/

/**
 * @brief [TODO:description]
 *
 * @param dev [TODO:parameter]
 * @return [TODO:return]
 */
static int gpio_emul_net_init(const struct device *dev) {

  struct gpio_emul_net_data *data = GPIO_EMUL_DATA(dev->data);

  memset(&data->server_addr, 0, sizeof(struct sockaddr_un));

  data->server_addr.sun_family = AF_UNIX;
  strncpy(data->server_addr.sun_path, prv_default_path,
          sizeof(data->server_addr.sun_path) - 1);

  data->thread_id = k_thread_create(
      &data->thread, data->stack, data->stack_size, prv_thread_handler,
      (void *)dev, NULL, NULL, GPIO_EMUL_NET_THREAD_PRIORITY, 0, K_NO_WAIT);

  const struct gpio_driver_config *parent_config = data->parent->config;

  return 0;
}

/*****************************************************************************
 * Public Functions
 *****************************************************************************/

/*****************************************************************************
 * Device Tree Bindings
 *****************************************************************************/

#define GPIO_NET_DEFINE(inst)                                                  \
  BUILD_ASSERT(                                                                \
      DT_NODE_HAS_COMPAT_STATUS(DT_INST_PARENT(inst), zephyr_gpio_emul, okay), \
      "Enable parent zephyr,gpio-emul node is required");                      \
                                                                               \
  static K_THREAD_STACK_DEFINE(gpio_emul_net_##inst##_stack,                   \
                               GPIO_EMUL_NET_THREAD_STACK_SIZE_BYTES);         \
                                                                               \
  static struct gpio_emul_net_data data_##inst = {                             \
      .stack = &gpio_emul_net_##inst##_stack,                                  \
      .stack_size = K_THREAD_STACK_SIZEOF(gpio_emul_net_##inst##_stack),       \
      .parent = DEVICE_DT_GET(DT_INST_PARENT(inst))};                          \
                                                                               \
  static const struct gpio_emul_net_config gpio_net_##inst##_config = {        \
      .data = &data_##inst,                                                    \
  };                                                                           \
                                                                               \
  DEVICE_DT_INST_DEFINE(inst, gpio_emul_net_init, NULL, &data_##inst,          \
                        &gpio_net_##inst##_config, POST_KERNEL,                \
                        CONFIG_GPIO_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(GPIO_NET_DEFINE)
