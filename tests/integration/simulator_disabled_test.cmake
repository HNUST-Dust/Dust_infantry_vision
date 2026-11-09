set(config_path "${CMAKE_CURRENT_BINARY_DIR}/simulator_disabled_test.yaml")
file(WRITE "${config_path}" "runtime:\n  io_backend: talos\n")
execute_process(
  COMMAND "${INFANTRY}" "${config_path}"
  RESULT_VARIABLE exit_code
  OUTPUT_VARIABLE stdout
  ERROR_VARIABLE stderr
  TIMEOUT 5
)
file(REMOVE "${config_path}")
if(NOT exit_code STREQUAL "2")
  message(FATAL_ERROR "Expected exit code 2, got ${exit_code}: ${stdout}${stderr}")
endif()
if(NOT "${stdout}${stderr}" MATCHES "Talos simulator backend is not compiled in")
  message(FATAL_ERROR "Missing simulator build diagnostic: ${stdout}${stderr}")
endif()
