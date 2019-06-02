set(COMPONENT_SRCS
    main.c
    mdns.c
    mqtt.c
)
set(COMPONENT_ADD_INCLUDEDIRS "")
set(COMPONENT_PRIV_REQUIRES
    crosslog
    uev
    nvs_flash
    mdns
    mqttc
)

register_component()
