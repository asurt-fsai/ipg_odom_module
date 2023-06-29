#include_directories()

set(PID_SRCS
    PID.h PID.cpp
    )
    
SET(LIB_TYPE STATIC)

add_library (pid ${LIB_TYPE} ${PID_SRCS})
