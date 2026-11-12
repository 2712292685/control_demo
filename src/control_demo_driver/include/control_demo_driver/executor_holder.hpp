#ifndef CONTROL_DEMO_DRIVER__EXECUTOR_HOLDER_HPP_
#define CONTROL_DEMO_DRIVER__EXECUTOR_HOLDER_HPP_

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include "zsibot_sdk/zsibot_api.h"

namespace control_demo_driver
{

/// 在后台线程创建 ZsibotExecutor。
///
/// SDK 构造函数会阻塞到收到机器狗 model 信息为止（内部 10 ms 轮询），
/// 因此不能直接在 ROS 定时器或构造函数中同步创建。该类通过 detached
/// 线程完成初始化，并用 shared_ptr 保证线程比节点活得更久时不会悬空。
class ExecutorHolder : public std::enable_shared_from_this<ExecutorHolder>
{
   public:
    ExecutorHolder() = default;
    ~ExecutorHolder();

    ExecutorHolder(const ExecutorHolder &) = delete;
    ExecutorHolder & operator=(const ExecutorHolder &) = delete;

    using TimePoint = std::chrono::steady_clock::time_point;

    /// 若 executor 已就绪返回 true；否则按需启动后台创建并返回 false。
    bool ensure(zsibot::Role role, const std::string & ip, uint32_t send_port,
                uint32_t recv_port, TimePoint now);

    /// 获取 executor 裸指针（生命周期到 shutdown 为止）。
    zsibot::ZsibotExecutor * get();

    /// 若上一次后台创建失败，返回 true 并取出错误信息（只返回一次）。
    bool take_failure(std::string & error);

    bool in_progress() const;

    /// 停止并销毁 executor（会 join SDK 线程）。
    void shutdown();

   private:
    mutable std::mutex mutex_;
    std::unique_ptr<zsibot::ZsibotExecutor> executor_;
    bool init_in_progress_{false};
    bool init_failed_{false};
    bool stop_requested_{false};
    std::string init_error_;
    TimePoint next_retry_{};
};

}  // namespace control_demo_driver

#endif  // CONTROL_DEMO_DRIVER__EXECUTOR_HOLDER_HPP_
