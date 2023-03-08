add_library(smbus STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/smbus.c
    ${CMAKE_CURRENT_SOURCE_DIR}/smbus.h
)

target_include_directories(smbus PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Must link against libi2c
target_link_libraries(smbus i2c)
