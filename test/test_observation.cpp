#include "mini_vo/core/observation.h"

#include <cassert>
#include <iostream>
#include <limits>

int main() {
    mini_vo::ObservationStore store;

    const mini_vo::Observation first{1, 10, 3, {120.0f, 80.0f}, 0, false};
    const mini_vo::Observation second{2, 10, 7, {118.0f, 81.0f}, 1, false};

    assert(store.add(first));
    assert(store.add(second));
    assert(store.size() == 2);
    assert(store.byMapPoint(10).size() == 2);
    assert(store.byKeyFrame(1).size() == 1);

    assert(!store.add(first));

    auto invalid = first;
    invalid.map_point_id = 11;
    invalid.pixel.x = std::numeric_limits<float>::quiet_NaN();
    assert(!store.add(invalid));

    assert(store.erase(1, 10));
    assert(store.size() == 1);
    assert(store.validate());

    std::cout << "[PASS] observation store\n";
    return 0;
}
