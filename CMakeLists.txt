cmake_minimum_required(VERSION 3.10)
project(GamePhysics)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 解决 OpenGL 警告
cmake_policy(SET CMP0072 NEW)

# 公共库
add_library(common STATIC
    common/Vec3.cpp
    common/Mat3.cpp
    common/AABB.cpp
    common/Collider.cpp
    common/RigidBody.cpp
)
target_include_directories(common PUBLIC common)

# 宽阶段库
add_library(broadphase_lib STATIC
    stage2_broadphase/Broadphase.cpp
)
target_include_directories(broadphase_lib PUBLIC stage2_broadphase)
target_link_libraries(broadphase_lib common)

# GJK/EPA 库
add_library(gjk_epa_lib STATIC
    stage3_detection/GJK.cpp
    stage3_detection/EPA.cpp
)
target_include_directories(gjk_epa_lib PUBLIC stage3_detection)
target_link_libraries(gjk_epa_lib common broadphase_lib)

# 约束求解器库 - 注意 stage4 和 stage5 使用不同版本
# stage4 使用 stage4_resolution/ContactConstraint.cpp
add_library(constraint4_lib STATIC
    stage4_resolution/ContactConstraint.cpp
)
target_include_directories(constraint4_lib PUBLIC stage4_resolution)
target_link_libraries(constraint4_lib common gjk_epa_lib)

# stage5 使用 stage5_stability/ContactConstraint.cpp (带热启动)
add_library(constraint5_lib STATIC
    stage5_stability/ContactConstraint.cpp
)
target_include_directories(constraint5_lib PUBLIC stage5_stability)
target_link_libraries(constraint5_lib common gjk_epa_lib)

# ============================================
# OpenGL 渲染器库
# ============================================

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

add_library(renderer STATIC
    common/OpenGLRenderer.cpp
)
target_link_libraries(renderer common ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES})
target_include_directories(renderer PUBLIC common)

# ============================================
# 命令行版本
# ============================================

# 阶段1
add_executable(stage1_motion stage1_motion/main.cpp)
target_link_libraries(stage1_motion common)

# 阶段2
add_executable(stage2_broadphase 
    stage2_broadphase/main.cpp
    stage2_broadphase/PhysicsWorld.cpp
)
target_link_libraries(stage2_broadphase common broadphase_lib)

# 阶段3
add_executable(stage3_detection 
    stage3_detection/main.cpp
    stage3_detection/PhysicsWorld.cpp
)
target_link_libraries(stage3_detection common gjk_epa_lib)

# 阶段4 - 需要链接 constraint4_lib
add_executable(stage4_resolution 
    stage4_resolution/main.cpp
    stage4_resolution/PhysicsWorld.cpp
    stage4_resolution/ContactConstraint.cpp
)
target_link_libraries(stage4_resolution common gjk_epa_lib constraint4_lib)

# 阶段5 - 需要链接 constraint5_lib
add_executable(stage5_stability 
    stage5_stability/main.cpp
    stage5_stability/PhysicsWorld.cpp
    stage5_stability/ContactConstraint.cpp
)
target_link_libraries(stage5_stability common gjk_epa_lib constraint5_lib)

# 阶段6
add_executable(stage6_polyhedron 
    stage6_polyhedron/main.cpp
    stage6_polyhedron/PhysicsWorld.cpp
    stage6_polyhedron/HalfEdge.cpp
    stage6_polyhedron/PolyhedronCollider.cpp
)
target_link_libraries(stage6_polyhedron common constraint5_lib)

# ============================================
# 可视化版本
# ============================================

# 阶段1 可视化
add_executable(stage1_visible stage1_motion/main_visible.cpp)
target_link_libraries(stage1_visible renderer common)

# 阶段2 可视化
add_executable(stage2_visible 
    stage2_broadphase/main_visible.cpp
    stage2_broadphase/PhysicsWorld.cpp
)
target_link_libraries(stage2_visible renderer common broadphase_lib)

# 阶段3 可视化
add_executable(stage3_visible 
    stage3_detection/main_visible.cpp
    stage3_detection/PhysicsWorld.cpp
)
target_link_libraries(stage3_visible renderer common gjk_epa_lib)

# 阶段4 可视化
add_executable(stage4_visible 
    stage4_resolution/main_visible.cpp
    stage4_resolution/PhysicsWorld.cpp
    stage4_resolution/ContactConstraint.cpp
)
target_link_libraries(stage4_visible renderer common gjk_epa_lib constraint4_lib)

# 阶段5 可视化
add_executable(stage5_visible 
    stage5_stability/main_visible.cpp
    stage5_stability/PhysicsWorld.cpp
    stage5_stability/ContactConstraint.cpp
)
target_link_libraries(stage5_visible renderer common gjk_epa_lib constraint5_lib)

# 阶段6 可视化
add_executable(stage6_visible 
    stage6_polyhedron/main_visible.cpp
    stage6_polyhedron/PhysicsWorld.cpp
    stage6_polyhedron/HalfEdge.cpp
    stage6_polyhedron/PolyhedronCollider.cpp
)
target_link_libraries(stage6_visible renderer common constraint5_lib)