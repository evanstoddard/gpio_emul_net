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

#include <poll.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <gpio_emul_net/gpio_emul_net_protocol.h>
#include <gpio_emul_net/gpio_emul_net_protocol_handler.h>

/*****************************************************************************
 * Definitions
 *****************************************************************************/

LOG_MODULE_REGISTER(gpio_emul_net, CONFIG_GPIO_LOG_LEVEL);

#define GPIO_EMUL_DATA(ptr) ((struct gpio_emul_net_data *)ptr)

#define GPIO_EMUL_CONFIG(ptr) ((struct gpio_emul_net_config *)ptr)

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

  uint32_t num_gpios;
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
 * @param data [TODO:parameter]
 * @param size [TODO:parameter]
 * @param ctx [TODO:parameter]
 * @return [TODO:return]
 */
static int prv_socket_write_function(const void *data, size_t size, void *ctx) {
  const struct device *dev = (const struct device *)ctx;
  struct gpio_emul_net_data *inst_data = GPIO_EMUL_DATA(dev->data);

  if (inst_data->server_fd < 0) {
    return -ENODEV;
  }

  ssize_t written = write(inst_data->server_fd, data, size);

  if (written < 0) {
    return -errno;
  }

  if (written < 0) {
    return -ECONNABORTED;
  }

  if (written != size) {
    return -EBADMSG;
  }

  return 0;
}

/**
 * @brief [TODO:description]
 *
 * @param dev [TODO:parameter]
 */
static void prv_write_pin_flags(const struct device *dev) {
  struct gpio_emul_net_data *data = GPIO_EMUL_DATA(dev->data);

  LOG_INF("Start write pin flags.");

  for (uint32_t i = 0; i < data->num_gpios; i++) {
    gpio_flags_t flags = 0;

    int ret = gpio_emul_flags_get(data->parent, i, &flags);

    if (ret < 0) {
      LOG_ERR("Failed to get pin flags: %d", ret);
      break;
    }

    ret = gpio_net_emul_write_gpio_flags_message(i, flags);
    if (ret < 0) {
      LOG_ERR("Failed to write pin flags message: %d", ret);
      break;
    }
  }
}

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
 * @param dev [TODO:parameter]
 * @param pollfd [TODO:parameter]
 * @return [TODO:return]
 */
static int prv_handle_server_event(const struct device *dev,
                                   const struct pollfd *pollfd) {

  if (pollfd->revents & POLLPRI || pollfd->revents & POLLHUP) {
    return -ECONNABORTED;
  }

  return 0;
}

/**
 * @brief [TODO:description]
 *
 * @param dev [TODO:parameter]
 */
static void prv_poll_server(const struct device *dev) {
  struct gpio_emul_net_data *data = GPIO_EMUL_DATA(dev->data);

  struct pollfd server_pollfd = {0};
  server_pollfd.fd = data->server_fd;
  server_pollfd.events = POLLIN | POLLPRI | POLLHUP;

  while (true) {
    int ret = poll(&server_pollfd, 1, -1);

    if (ret < 0) {
      LOG_ERR("Server poll error: %d", ret);
      return;
    }

    ret = prv_handle_server_event(dev, &server_pollfd);

    if (ret < 0) {
      return;
    }
  }
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
      LOG_INF("Connected to server.");

      ret = gpio_emul_net_write_ident_message(data->num_gpios);

      if (ret < 0) {
        LOG_ERR("Failed to write identify message: %d", ret);
      }

      prv_write_pin_flags(dev);

      prv_poll_server(dev);

      close(data->server_fd);
      data->server_fd = -1;
      LOG_WRN("Disconnected from server.");
    }
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

  gpio_emul_net_set_write_function(
      prv_socket_write_function,
      (void *)(void *)(void *)(void *)(void *)(void *)(void *)(void *)(void *)
          dev);

  data->thread_id = k_thread_create(
      &data->thread, data->stack, data->stack_size, prv_thread_handler,
      (void *)dev, NULL, NULL, GPIO_EMUL_NET_THREAD_PRIORITY, 0, K_NO_WAIT);

  const struct gpio_driver_config *parent_config = data->parent->config;
  data->num_gpios = prv_num_gpios_from_bitmask(parent_config->port_pin_mask);

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
