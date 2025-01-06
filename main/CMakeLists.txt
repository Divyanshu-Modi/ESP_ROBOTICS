#
# ESP ROBOTICS
#
set(TSRCS "main.c")
set(DIRS "include/")

if (CONFIG_CYTRON_MDDS)
	list(APPEND TSRCS "cytron/mdds.c")
	list(APPEND DIRS "cytron/")
endif()

if (CONFIG_CYTRON_LSA)
	list(APPEND TSRCS "cytron/lsa08.c")
	list(APPEND DIRS "cytron/")
endif()

if (CONFIG_ECONNECT_SPI)
	list(APPEND TSRCS "econnect/espi.c")
	list(APPEND DIRS "econnect/")
endif()

if (CONFIG_ROTARY_ENCODER)
	list(APPEND TSRCS "encoder/rotary_encoder.c")
	list(APPEND DIRS "encoder/")
endif()

idf_component_register(SRCS ${TSRCS} INCLUDE_DIRS ${DIRS})
