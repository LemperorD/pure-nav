#include "shm.hpp"

#include "shm_internal.hpp"

#include <memory>
#include <stdexcept>
#include <string>

#include <iceoryx_posh/iceoryx_posh_config.hpp>
#include <iceoryx_posh/internal/roudi/roudi.hpp>
#include <iceoryx_posh/mepoo/mepoo_config.hpp>
#include <iceoryx_posh/roudi/iceoryx_roudi_components.hpp>
#include <iceoryx_posh/runtime/posh_runtime_single_process.hpp>
#include <iox/posix_group.hpp>

namespace pure {
namespace common {

namespace {

// RouDi 及其依赖必须活到进程结束；用函数内静态对象持有，析构顺序与构造相反
// （runtime -> roudi -> components），与 iceoryx 官方 singleprocess 示例一致。
struct SingleProcessState {
    std::unique_ptr<iox::roudi::IceOryxRouDiComponents> components;
    std::unique_ptr<iox::roudi::RouDi> roudi;
    std::unique_ptr<iox::runtime::PoshRuntimeSingleProcess> runtime;

    ~SingleProcessState() {
        runtime.reset();
        roudi.reset();
        components.reset();
    }
};

SingleProcessState& singleProcessState() {
    static SingleProcessState state;
    return state;
}

iox::IceoryxConfig makeIceoryxConfig(const ShmRuntimeConfig& config) {
    iox::IceoryxConfig iceoryx_config = iox::IceoryxConfig().setDefaults();

    if (config.mempools.empty()) {
        return iceoryx_config;
    }

    iox::mepoo::MePooConfig mempool_config;
    for (const ShmMemPool& pool : config.mempools) {
        if (pool.chunk_size == 0 || pool.chunk_count == 0) {
            throw std::invalid_argument("ShmMemPool 的 chunk_size / chunk_count 必须大于 0");
        }
        mempool_config.addMemPool({pool.chunk_size, pool.chunk_count});
    }

    const auto group = iox::PosixGroup::getGroupOfCurrentProcess().getName();
    iceoryx_config.m_sharedMemorySegments.clear();
    iceoryx_config.m_sharedMemorySegments.push_back({group, group, mempool_config});
    return iceoryx_config;
}

} // namespace

void ShmRuntime::initSingleProcess(const std::string& app_name, const ShmRuntimeConfig& config) {
    if (detail::runtimeInitialized()) {
        return;
    }

    const std::string effective_name = app_name.empty() ? defaultAppName() : app_name;

    iox::IceoryxConfig iceoryx_config = makeIceoryxConfig(config);
    // RouDi 与本进程共享地址空间：RouDi 先于应用退出时不要顺手把应用也结束掉
    iceoryx_config.sharesAddressSpaceWithApplications = true;

    SingleProcessState& state = singleProcessState();
    state.components = std::make_unique<iox::roudi::IceOryxRouDiComponents>(iceoryx_config);
    state.roudi = std::make_unique<iox::roudi::RouDi>(state.components->rouDiMemoryManager,
                                                      state.components->portManager,
                                                      iceoryx_config);
    state.runtime = std::make_unique<iox::runtime::PoshRuntimeSingleProcess>(detail::makeRuntimeName(effective_name));

    detail::markRuntimeInitialized();
}

} // namespace common
} // namespace pure
