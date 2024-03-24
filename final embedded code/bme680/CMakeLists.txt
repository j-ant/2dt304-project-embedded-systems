cmake_minimum_required(VERSION 3.10)

# Create a static library for BME680
add_library(bme680 STATIC
    bme68x.c
)

# Include directory for BME680 library
target_include_directories(bme680 PUBLIC
    include
)
