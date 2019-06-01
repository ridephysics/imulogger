set(COMPONENT_SRCS "main.c")
set(COMPONENT_ADD_INCLUDEDIRS "")
set(COMPONENT_PRIV_REQUIRES
    crosslog
    uev
    nvs_flash
)

register_component()
