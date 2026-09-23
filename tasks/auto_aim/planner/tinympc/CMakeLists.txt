add_library(tinympcstatic STATIC
    admm.cpp
    tiny_api.cpp
    codegen.cpp
    rho_benchmark.cpp
)

set_target_properties(tinympcstatic PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(tinympcstatic PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..)
