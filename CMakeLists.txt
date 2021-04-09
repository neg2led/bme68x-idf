file(GLOB component_srcs BME68x-Sensor-API/*.c)

set(component_include "BME68x-Sensor-API" "src")

idf_component_register(
    SRCS component_srcs
    PRIV_REQUIRES "driver"
    INCLUDE_DIRS component_include
)
