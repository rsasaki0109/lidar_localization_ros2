#include "tf_single_wait.hpp"
#include <cassert>
#include <stdexcept>
#include <thread>
using namespace tf_single_wait;
int main() {
  assert(timeoutSeconds()==0.1 && timeoutSeconds()==0.1);
  {
    Scope scan;
    assert(timeoutSeconds()==0.1 && timeoutSeconds()==0.0);
    std::thread other([] {
      assert(timeoutSeconds()==0.1 && timeoutSeconds()==0.1);
      Scope scan;
      assert(timeoutSeconds()==0.1 && timeoutSeconds()==0.0);
    });
    other.join();
    try {
      Scope nested;
      assert(timeoutSeconds()==0.1 && timeoutSeconds()==0.0);
      throw std::runtime_error("unwind");
    } catch (const std::runtime_error &) {}
    assert(timeoutSeconds()==0.0);
  }
  assert(timeoutSeconds()==0.1 && timeoutSeconds()==0.1);
  {Scope next_scan; assert(timeoutSeconds()==0.1 && timeoutSeconds()==0.0);}
}
