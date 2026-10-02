#pragma once
#include <mutex>

namespace ec::host {
// 仅用于主机验证, 支持任务/线程同步; 不可用于 ISR.
class MutexCriticalSection {
public:
    [[nodiscard]] unsigned enter() noexcept { mutex_.lock(); return 0; }
    void exit(unsigned) noexcept { mutex_.unlock(); }
private:
    std::mutex mutex_;
};
} // 命名空间 ec::host
