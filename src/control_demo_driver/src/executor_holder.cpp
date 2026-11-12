#include "control_demo_driver/executor_holder.hpp"

#include <exception>
#include <thread>
#include <utility>

namespace control_demo_driver
{

ExecutorHolder::~ExecutorHolder()
{
    shutdown();
}

bool ExecutorHolder::ensure(
    zsibot::Role role, const std::string & ip, uint32_t send_port, uint32_t recv_port,
    TimePoint now)
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (executor_) {
            return true;
        }
        if (init_in_progress_) {
            return false;
        }
        if (now < next_retry_) {
            return false;
        }
        init_in_progress_ = true;
        init_failed_ = false;
    }

    std::shared_ptr<ExecutorHolder> self;
    try {
        self = shared_from_this();
    } catch (const std::exception & exception) {
        std::lock_guard<std::mutex> lock(mutex_);
        init_in_progress_ = false;
        init_error_ = exception.what();
        init_failed_ = true;
        next_retry_ = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        return false;
    }

    try {
        std::thread([self, role, ip, send_port, recv_port]() {
            try {
                auto exec = std::make_unique<zsibot::ZsibotExecutor>(
                    role, ip, send_port, recv_port);
                std::lock_guard<std::mutex> lock(self->mutex_);
                if (self->stop_requested_) {
                    // 节点已退出：exec 在此线程内析构并 join SDK 线程。
                    self->init_in_progress_ = false;
                    return;
                }
                self->executor_ = std::move(exec);
                self->init_in_progress_ = false;
            } catch (const std::exception & exception) {
                std::lock_guard<std::mutex> lock(self->mutex_);
                self->init_error_ = exception.what();
                self->init_failed_ = true;
                self->init_in_progress_ = false;
                self->next_retry_ =
                    std::chrono::steady_clock::now() + std::chrono::seconds(5);
            }
        }).detach();
    } catch (const std::exception & exception) {
        std::lock_guard<std::mutex> lock(mutex_);
        init_in_progress_ = false;
        init_error_ = exception.what();
        init_failed_ = true;
        next_retry_ = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    }
    return false;
}

zsibot::ZsibotExecutor * ExecutorHolder::get()
{
    std::lock_guard<std::mutex> lock(mutex_);
    return executor_.get();
}

bool ExecutorHolder::take_failure(std::string & error)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!init_failed_) {
        return false;
    }
    init_failed_ = false;
    error = init_error_;
    return true;
}

bool ExecutorHolder::in_progress() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return init_in_progress_;
}

void ExecutorHolder::shutdown()
{
    std::lock_guard<std::mutex> lock(mutex_);
    stop_requested_ = true;
    executor_.reset();
}

}  // namespace control_demo_driver
