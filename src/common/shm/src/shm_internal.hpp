#pragma once

// ============================================================================
// shm_internal.hpp —— pure_nav_shm 内部共享的接口（不对外暴露）
// ============================================================================

#include <string>

#include <iceoryx_posh/iceoryx_posh_types.hpp>

namespace pure {
namespace common {
namespace detail {

// iceoryx 一个进程只能初始化一次运行时；用进程内标志记录，避免重复 init。
bool runtimeInitialized() noexcept;
void markRuntimeInitialized() noexcept;

// 校验并转换 iceoryx 运行时名（超长抛 std::invalid_argument）。
iox::RuntimeName_t makeRuntimeName(const std::string& name);

} // namespace detail
} // namespace common
} // namespace pure
