set(COMPONENT_SRCS
    main.c
    mdns.c
    mqtt.c
    usfs.c
)
set(COMPONENT_ADD_INCLUDEDIRS "")
set(COMPONENT_PRIV_REQUIRES
    crosslog
    crossi2c
    em7180
    bmp280
    eMPL
    usfs
    uev
    nvs_flash
    mdns
    mqttc
    fatfs
    uftpd
)

register_component()
