#include "tf_shared_wait.hpp"
#include <cassert>
#include <stdexcept>
#include <thread>
using namespace tf_shared_wait;
int main() {
  static_assert(shared_timeout_sec == 2 * original_timeout_sec);
  assert(lookupTimeoutSeconds()==0.1);
  {
    Scope scan(true);
    assert(lookupTimeoutSeconds()==0.0 && lookupTimeoutSeconds()==0.0);
    std::thread other([] {
      assert(lookupTimeoutSeconds()==0.1);
      {Scope scan(true); assert(lookupTimeoutSeconds()==0.0);}
      assert(lookupTimeoutSeconds()==0.1);
    });
    other.join();
    try {
      Scope disabled(false);
      assert(lookupTimeoutSeconds()==0.1);
      throw std::runtime_error("unwind");
    } catch (const std::runtime_error &) {}
    assert(lookupTimeoutSeconds()==0.0);
  }
  assert(lookupTimeoutSeconds()==0.1);
  {Scope disabled(false); assert(lookupTimeoutSeconds()==0.1);}
  {Scope next_scan(true); assert(lookupTimeoutSeconds()==0.0);}
  assert(lookupTimeoutSeconds()==0.1);
}
