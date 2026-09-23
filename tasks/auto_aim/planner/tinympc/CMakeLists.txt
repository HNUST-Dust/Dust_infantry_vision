add_library(tinympcstatic STATIC
    admm.cpp
    tiny_api.cpp
    codegen.cpp
    rho_benchmark.cpp
)

set_target_properties(tinympcstatic PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(tinympcstatic PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..)

# types.hpp 含 <Eigen/Eigen>、codegen.cpp 含 <Eigen/Dense>。阶段 1 删掉的那条指向
# 不存在目录的 -I 在这里补上正确写法。
target_link_libraries(tinympcstatic PUBLIC Eigen3::Eigen)
