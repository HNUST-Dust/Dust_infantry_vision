# generated from
# rosidl_cmake/cmake/template/rosidl_cmake_export_typesupport_targets.cmake.in

set(_exported_typesupport_targets
  "__rosidl_generator_c:rm_decision_interfaces__rosidl_generator_c;__rosidl_typesupport_fastrtps_c:rm_decision_interfaces__rosidl_typesupport_fastrtps_c;__rosidl_typesupport_introspection_c:rm_decision_interfaces__rosidl_typesupport_introspection_c;__rosidl_typesupport_c:rm_decision_interfaces__rosidl_typesupport_c;__rosidl_generator_cpp:rm_decision_interfaces__rosidl_generator_cpp;__rosidl_typesupport_fastrtps_cpp:rm_decision_interfaces__rosidl_typesupport_fastrtps_cpp;__rosidl_typesupport_introspection_cpp:rm_decision_interfaces__rosidl_typesupport_introspection_cpp;__rosidl_typesupport_cpp:rm_decision_interfaces__rosidl_typesupport_cpp;__rosidl_generator_py:rm_decision_interfaces__rosidl_generator_py")

# populate rm_decision_interfaces_TARGETS_<suffix>
if(NOT _exported_typesupport_targets STREQUAL "")
  # loop over typesupport targets
  foreach(_tuple ${_exported_typesupport_targets})
    string(REPLACE ":" ";" _tuple "${_tuple}")
    list(GET _tuple 0 _suffix)
    list(GET _tuple 1 _target)

    set(_target "rm_decision_interfaces::${_target}")
    if(NOT TARGET "${_target}")
      # the exported target must exist
      message(WARNING "Package 'rm_decision_interfaces' exports the typesupport target '${_target}' which doesn't exist")
    else()
      list(APPEND rm_decision_interfaces_TARGETS${_suffix} "${_target}")
    endif()
  endforeach()
endif()
