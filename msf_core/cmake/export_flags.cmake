add_definitions (-Wall -std=c++11 -Wno-unused-parameter -O3)# -Wextra -pedantic")

# assembler on mac os doesn't know avx commands :( switch to sse4.2. assuming that our mac machines have a corei7
IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 
  add_definitions (-msse4.2)
ELSE(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  add_definitions (-march=native)
ENDIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

