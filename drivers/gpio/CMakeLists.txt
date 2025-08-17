target_sources(app PRIVATE
  gpio_emul_net.c
)

target_link_libraries(app PRIVATE
  gpio_emul_net_protocol
)
