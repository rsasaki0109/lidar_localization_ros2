#include "tf_lookup_scope.hpp"
#include <cassert>
#include <stdexcept>
#include <thread>
using namespace tf_lookup_scope;
int main() {
  assert(timeoutSeconds()==0.1);
  {
    Scope scan(true);
    assert(timeoutSeconds()==0.0 && timeoutSeconds()==0.0);
    std::thread other([] {
      assert(timeoutSeconds()==0.1);
      {Scope scan(true); assert(timeoutSeconds()==0.0);}
      assert(timeoutSeconds()==0.1);
    });
    other.join();
    try {
      Scope disabled(false);
      assert(timeoutSeconds()==0.1);
      throw std::runtime_error("unwind");
    } catch (const std::runtime_error &) {}
    assert(timeoutSeconds()==0.0);
  }
  assert(timeoutSeconds()==0.1);
  {Scope disabled(false); assert(timeoutSeconds()==0.1);}
  {Scope next_scan(true); assert(timeoutSeconds()==0.0);}
  assert(timeoutSeconds()==0.1);
}
