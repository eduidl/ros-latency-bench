#pragma once

#include <chrono>
#include <thread>

namespace latency_bench
{
class DummyProcess
{
public:
  explicit DummyProcess(int sleep_us) : should_sleep_(sleep_us > 0), sleep_us_(sleep_us) {}
  ~DummyProcess() = default;

  void execute() const
  {
    if (should_sleep_) {
      std::this_thread::sleep_for(sleep_us_);
    }
  }

private:
  bool should_sleep_;
  std::chrono::microseconds sleep_us_;
};

}  // namespace latency_bench
