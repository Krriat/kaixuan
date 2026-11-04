execute_process(COMMAND "${PROGRAM}"
  INPUT_FILE "${INPUT}"
  OUTPUT_VARIABLE output
  ERROR_VARIABLE diagnostics
  RESULT_VARIABLE result)
if(NOT result EQUAL 0)
  message(FATAL_ERROR "CLI failed: ${diagnostics}")
endif()
string(STRIP "${output}" output)
string(REGEX REPLACE "[ \t\r\n]+" ";" values "${output}")
list(LENGTH values count)
if(NOT count EQUAL 3 OR NOT diagnostics MATCHES "status=solved")
  message(FATAL_ERROR "Expected three joint velocities, got: ${output}\n${diagnostics}")
endif()
foreach(value IN LISTS values)
  if(NOT value MATCHES "^[-+0-9.eE]+$" OR value LESS 0.19999 OR value GREATER 0.20001)
    message(FATAL_ERROR "Expected saturated joint velocity 0.2, got ${value}")
  endif()
endforeach()
