# ============================================================================
# thirdparty —— 第三方库
#
# 组织约定：本目录不再向下拆分 CMakeLists，第三方库一律直接写在本文件里。
#
# header-only 库（例如把 doctest.h 放进 thirdparty/doctest/）：
#   add_library(doctest INTERFACE)
#   target_include_directories(doctest INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/doctest")
#
# 需要编译源码的库：
#   add_library(xxx STATIC
#       xxx/src/a.cpp
#       xxx/src/b.cpp)
#   target_include_directories(xxx PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/xxx/include")
#
# 当前没有任何第三方库，故本文件不含任何目标。
# ============================================================================
