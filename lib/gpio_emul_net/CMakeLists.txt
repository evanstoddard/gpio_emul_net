set(GPIO_EMUL_NET_LIB_INCLUDE_PATH
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

set(GPIO_EMUL_NET_LIB_INCLUDE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/include CACHE INTERNAL "GPIO EMUL NET Library include path.")

if(DEFINED ZEPHYR_BASE)
  target_include_directories(app PRIVATE ${GPIO_EMUL_NET_LIB_INCLUDE_PATH})
endif()

add_library(gpio_emul_net_protocol
  src/gpio_emul_net_protocol_handler.c
)

target_include_directories(gpio_emul_net_protocol PRIVATE include/gpio_emul_net)
