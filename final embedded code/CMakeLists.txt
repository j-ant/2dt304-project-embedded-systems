cmake_minimum_required(VERSION 3.12)

if (PICO_CYW43_SUPPORTED)
    if (NOT TARGET pico_cyw43_arch)
        message("Skipping Pico W examples as support is not available")
    else()

        if (DEFINED ENV{WIFI_SSID} AND (NOT WIFI_SSID))
            set(WIFI_SSID $ENV{WIFI_SSID})
            message("Using WIFI_SSID from environment ('${WIFI_SSID}')")
        endif()

        if (DEFINED ENV{WIFI_PASSWORD} AND (NOT WIFI_PASSWORD))
            set(WIFI_PASSWORD $ENV{WIFI_PASSWORD})
            message("Using WIFI_PASSWORD from environment")
        endif()

        set(WIFI_SSID "${WIFI_SSID}" CACHE INTERNAL "WiFi SSID for examples")
        set(WIFI_PASSWORD "${WIFI_PASSWORD}" CACHE INTERNAL "WiFi password for examples")

        add_subdirectory(wifi)
        if (NOT TARGET pico_btstack_base)
            message("Skipping Pico W Bluetooth examples as support is not available")
        else()
            add_subdirectory(bt)
        endif()
    endif()
endif()

include(pico_sdk_import.cmake)
project(air-quality)

pico_sdk_init()

add_definitions(-DWIFI_SSID="${WIFI_SSID}")
add_definitions(-DWIFI_PASSWORD="${WIFI_PASSWORD}")


add_executable(air-quality
    src/main.c
)

FILE(GLOB FreeRTOS_src FreeRTOS-Kernel/*.c)
FILE(GLOB bme680_src bme680/*.c)
FILE(GLOB transport_src transport/*.c)

add_library( FreeRTOS STATIC
  ${FreeRTOS_src}
  FreeRTOS-Kernel/portable/GCC/ARM_CM0/port.c
  FreeRTOS-Kernel/portable/MemMang/heap_4.c
)

add_library( bme680 STATIC
  ${bme680_src}
)

target_include_directories(air-quality PUBLIC
    lwip
)

target_include_directories( FreeRTOS PUBLIC
    FreeRTOS-Kernel/include
    include/
    FreeRTOS-Kernel/portable/GCC/ARM_CM0
    )

target_include_directories( bme680 PUBLIC
  bme680/include
)

target_link_libraries( air-quality 
  pico_stdlib
  pico_cyw43_arch_none
  pico_cyw43_arch_lwip_threadsafe_background
  pico_lwip_mqtt
  hardware_gpio
  hardware_i2c
  FreeRTOS
  bme680
  )

pico_enable_stdio_usb(air-quality 1)
pico_enable_stdio_uart(air-quality 0)
pico_add_extra_outputs(air-quality)