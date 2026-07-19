#include "glim_prior_map_localizer/prior_map_overlap_policy.hpp"

#include <cassert>

namespace localizer = glim_prior_map_localizer;

int main()
{
  const auto empty = localizer::decidePriorMapOverlap(0, 0, 10, 0.1);
  assert(!empty.sufficient);
  assert(empty.fraction == 0.0);

  const auto too_few = localizer::decidePriorMapOverlap(9, 100, 10, 0.05);
  assert(!too_few.sufficient);

  const auto too_sparse = localizer::decidePriorMapOverlap(10, 100, 10, 0.11);
  assert(!too_sparse.sufficient);

  const auto acquired = localizer::decidePriorMapOverlap(10, 100, 10, 0.10);
  assert(acquired.sufficient);
  assert(acquired.fraction == 0.10);

  const auto reentered = localizer::decidePriorMapOverlap(32, 320, 32, 0.05);
  assert(reentered.sufficient);
  return 0;
}
