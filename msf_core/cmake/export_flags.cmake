if (CMAKE_COMPILER_IS_GNUCC)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wno-unused-parameter")

# assembler on mac os doesn't know avx commands :( switch to sse4.2. assuming that our mac machines have a corei7
IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse4.2")
ELSE(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
ENDIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

