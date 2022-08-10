cmake_minimum_required(VERSION 2.8)

PROJECT(atenl C)
ADD_DEFINITIONS(-Os -Wall --std=gnu99 -g3)

ADD_EXECUTABLE(atenl main.c eth.c hqa.c nl.c eeprom.c util.c)
TARGET_LINK_LIBRARIES(atenl nl-tiny)

SET(CMAKE_INSTALL_PREFIX /usr)

INSTALL(TARGETS atenl
	RUNTIME DESTINATION sbin
)
