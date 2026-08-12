#ifdef NDEBUG
#undef NDEBUG
#endif

#include <cassert>
#include <chrono>
#include <future>
#include <optional>

#include "tools/ordered_delivery.hpp"

using namespace std::chrono_literals;

int main()
{
  tools::OrderedDelivery<int> delivery(3);
  assert(delivery.reserve(0));
  assert(delivery.reserve(1));
  assert(delivery.reserve(2));
  assert(!delivery.reserve(3));

  assert(delivery.complete(2, 22));
  assert(delivery.complete(0, 0));
  auto first = delivery.wait_pop();
  assert(first && first->sequence == 0 && first->value == 0);

  assert(delivery.skip(1, -1));
  auto second = delivery.wait_pop();
  auto third = delivery.wait_pop();
  assert(second && second->sequence == 1 && second->value == -1);
  assert(third && third->sequence == 2 && third->value == 22);

  assert(delivery.reserve(3));
  auto waiter = std::async(std::launch::async, [&] { return delivery.wait_pop(); });
  assert(waiter.wait_for(20ms) == std::future_status::timeout);
  delivery.close();
  assert(!delivery.reserve(4));
  assert(delivery.complete(3, 33));
  assert(waiter.wait_for(1s) == std::future_status::ready);
  auto final = waiter.get();
  assert(final && final->sequence == 3 && final->value == 33);
  assert(!delivery.wait_pop());

  return 0;
}
