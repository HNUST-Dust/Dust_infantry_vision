#include "exiter.hpp"

#include <csignal>
#include <stdexcept>

namespace tools
{
volatile std::sig_atomic_t exit_ = 0;
bool exiter_inited_ = false;

Exiter::Exiter()
{
  if (exiter_inited_) throw std::runtime_error("Multiple Exiter instances!");
  std::signal(SIGINT, [](int) { exit_ = 1; });
  exiter_inited_ = true;
}

bool Exiter::exit() const { return exit_ != 0; }

}  // namespace tools
