cmake_minimum_required(VERSION 3.20.0)

if(CONFIG_GPIO_EMUL_NET)
  add_subdirectory(drivers/gpio)
endif()

