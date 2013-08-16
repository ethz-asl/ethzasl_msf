SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11 -Wno-unused-parameter")# -Wextra -pedantic")

# assembler on mac os doesn't know avx commands :( switch to sse4.2. assuming that our mac machines have a corei7
IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse4.2")
ELSE(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
ENDIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

