#
# ESP ROBOTICS
#
set(TSRCS "main.c")
set(DIRS "include/")

if (CONFIG_CYTRON_MDDS)
	list(APPEND TSRCS "cytron/mdds.c")
	list(APPEND DIRS "cytron/")
endif()

idf_component_register(SRCS ${TSRCS} INCLUDE_DIRS ${DIRS})
