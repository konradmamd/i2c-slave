# i2c slave test
add_executable(test_slave
    ${CMAKE_CURRENT_SOURCE_DIR}/test_slave.c
    ${CMAKE_SOURCE_DIR}/include/smbus.h
    ${CMAKE_SOURCE_DIR}/include/i2c-slave-eeprom.h
)

target_include_directories(test_slave PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(test_slave)

# smbus master test
add_executable(test_master
    ${CMAKE_CURRENT_SOURCE_DIR}/test_master.c
    ${CMAKE_SOURCE_DIR}/include/smbus.h
)

target_include_directories(test_master PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(test_master smbus)
