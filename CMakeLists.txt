file(GLOB component_srcs BME68x-Sensor-API/*.c)

list(APPEND component_srcs "src/bme68x-idf.c")

set(component_include "BME68x-Sensor-API" "src")

idf_component_register(
    SRCS ${component_srcs}
    PRIV_REQUIRES "driver" "i2cdev"
    INCLUDE_DIRS ${component_include}
)
