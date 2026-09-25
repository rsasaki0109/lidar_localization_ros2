#include "tf_ready_queue.hpp"
#include <cassert>
#include <iostream>
#include <memory>
#include <vector>
using namespace std::chrono_literals;
using Q=tf_ready_queue::Queue<std::shared_ptr<const int>>;
const Q::Time zero{};
auto payload(int n) {return std::make_shared<const int>(n);}
int main() {
  const auto never=[](auto){return false;};
  const auto always=[](auto){return true;};
  bool threw=false;
  try {Q invalid(0,200ms,400ms);} catch(const std::invalid_argument&) {threw=true;}
  assert(threw);
  Q q(4,200ms,400ms);
  q.push(1,zero,payload(11)); q.push(2,zero+100ms,payload(22));
  assert(!q.take(zero+199ms,never));
  // A ready newer scan cannot overtake an older one awaiting TF.
  assert(!q.take(zero+150ms,[](auto stamp){return stamp==2;}));
  auto a=q.take(zero+200ms,never);
  assert(a && !a->tf_ready && a->entry.stamp==1 && *a->entry.payload==11);
  a=q.take(zero+200ms,always); assert(a && a->tf_ready && a->entry.stamp==2);
  assert(!q.push(2,zero+210ms,payload(0)) && q.counts().unordered==1);
  q.reset(); assert(q.push(1,zero+220ms,payload(33)));
  // Reset destroys queued snapshots; close cannot be reopened by reset.
  auto owned=payload(44); std::weak_ptr<const int> weak=owned;
  q.push(2,zero+230ms,std::move(owned)); q.reset(); assert(weak.expired());
  assert(q.counts().invalidated==2);
  for(int i=0;i<5;++i) {q.push(i,zero,payload(i));}
  assert(q.size()==4 && q.counts().overflow==1);
  a=q.take(zero+399ms,always); assert(a && a->entry.stamp==1);
  assert(!q.take(zero+400ms,always) && q.counts().stale==3);
  q.close(); q.reset(); assert(!q.push(100,zero,payload(1)));

  // Readiness errors must leave the front payload available for a later poll.
  Q errors(4,200ms,400ms); errors.push(1,zero,payload(1));
  threw=false;
  try {errors.take(zero,[](auto)->bool {throw std::runtime_error("TF error");});}
  catch(const std::runtime_error&) {threw=true;}
  assert(threw && errors.size()==1);
  assert(errors.take(zero,always)->entry.stamp==1);
  // A stalled registration worker does not permit unbounded retained clouds.
  Q overloaded(4,250ms,400ms);
  for(int ms=0;ms<1000;ms+=100) {
    overloaded.push(ms,zero+std::chrono::milliseconds(ms),payload(ms));
  }
  assert(overloaded.size()==4 && overloaded.counts().overflow==6);
  a=overloaded.take(zero+1000ms,always);
  assert(a && a->entry.stamp==700 && overloaded.counts().stale==1);
  overloaded.reset(); assert(!overloaded.take(zero+1100ms,always));

  // Deterministic 10Hz arrivals, 200ms delayed TF, independent 10ms polls.
  // Three seconds of unavailable TF exercise fallback without stalling intake.
  for(bool gap : {false,true}) {
    Q stream(4,250ms,400ms); std::vector<int> seen;
    int fallback=0; std::chrono::milliseconds max_age{0};
    for(int ms=0;ms<=10500;ms+=10) {
      auto now=zero+std::chrono::milliseconds(ms);
      if(ms<10000 && ms%100==0) {assert(stream.push(ms,now,payload(ms)));}
      auto out=stream.take(now,[&](auto stamp){
        return ms>=stamp+200 && !(gap && stamp>=3000 && stamp<6000);
      });
      if(out) {
        seen.push_back(static_cast<int>(out->entry.stamp));
        assert(*out->entry.payload==out->entry.stamp);
        fallback+=!out->tf_ready;
        max_age=std::max(max_age,std::chrono::duration_cast<std::chrono::milliseconds>(now-out->entry.received));
      }
      assert(stream.size()<=4);
    }
    assert(seen.size()==100 && stream.size()==0);
    for(int i=0;i<100;++i) {assert(seen[i]==i*100);}
    assert(fallback==(gap?30:0)); assert(max_age==(gap?250ms:200ms));
    assert(stream.counts().overflow==0 && stream.counts().stale==0);
    std::cout << "synthetic gap=" << gap << " dispatched=" << seen.size()
              << " fallback=" << fallback << " max_age_ms=" << max_age.count() << '\n';
  }
  std::cout << "queue contract assertions passed\n";
}
