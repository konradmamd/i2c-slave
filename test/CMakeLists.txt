# i2c slave test
add_executable(test_slave
    ${CMAKE_CURRENT_SOURCE_DIR}/test_slave.c
    ${CMAKE_CURRENT_SOURCE_DIR}/test_utils.c
    ${CMAKE_CURRENT_SOURCE_DIR}/test_utils.h
    ${CMAKE_SOURCE_DIR}/openbmc/i2c-slave-eeprom.h
)

target_include_directories(test_slave PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/openbmc
)

target_link_libraries(test_slave)

# smbus master test
add_executable(test_master
    ${CMAKE_CURRENT_SOURCE_DIR}/test_master.c
    ${CMAKE_CURRENT_SOURCE_DIR}/test_utils.c
    ${CMAKE_CURRENT_SOURCE_DIR}/test_utils.h
    ${CMAKE_SOURCE_DIR}/smbus/smbus.h
)

target_include_directories(test_master PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/smbus
)

target_link_libraries(test_master smbus)
