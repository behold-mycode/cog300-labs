#include "../Robot/LineFollower.h"
#include <cassert>
#include <initializer_list>

static void checkRecoveryProbe(int mask) {
  line::Follower follower;
  line::Input input;
  input.now = UINT32_MAX - 20;
  auto step = [&](int tape, uint32_t elapsed, float position) {
    input.tape = tape;
    input.now += elapsed;
    input.pose.left = input.pose.right = position;
    return follower.step(input);
  };
  follower.begin(input);
  step(0, 0, 0);
  auto probe = step(mask, 5, 0);
  step(mask, 5, 5);
  auto gap = step(0, 5, 10);
  assert(gap.left == probe.left && gap.right == probe.right);
  step(mask, 15, 26);
  assert(follower.state == line::Follower::VERIFY);
  step(mask, 10, 40);
  assert(follower.state == line::Follower::VERIFY);
  step(mask, 5, 41);
  assert(follower.state == line::Follower::FOLLOW);

  step(0, 50, 41);
  step(mask, 1, 41);
  step(0, 10, 46);
  step(mask, 10, 50);
  step(0, 24, 54);
  assert(follower.state == line::Follower::VERIFY);
  auto undo = step(0, 1, 55);
  assert(follower.state == line::Follower::UNDO_PROBE && undo.left < 0 && undo.right < 0);
  step(0, 10, 41);
  assert(follower.state == line::Follower::SEARCH);
}

int main() {
  for (int mask : {2, 3, 6}) checkRecoveryProbe(mask);
  for (int mask : {1, 2, 3, 4, 6}) {
    line::Follower follower;
    line::Input input;
    input.now = UINT32_MAX - 20;
    input.tape = mask;
    follower.begin(input);
    const auto before = follower.step(input);
    input.tape = 0;
    input.now += 44;
    const auto during = follower.step(input);
    assert(follower.state == line::Follower::FOLLOW);
    assert(during.left == before.left && during.right == before.right);
    assert(during.brakeInside == before.brakeInside);

    input.tape = 2;
    const auto centred = follower.step(input);
    assert(centred.left == follower.config.cruiseSpeed && centred.right == centred.left);
    input.tape = 0;
    input.now += 44;
    follower.step(input);
    assert(follower.state == line::Follower::FOLLOW);
    ++input.now;
    const auto search = follower.step(input);
    assert(follower.state == line::Follower::SEARCH && search.left * search.right < 0);

    input.tape = mask;
    follower.begin(input);
    follower.step(input);
    follower.stop();
    input.tape = 0;
    follower.begin(input);
    follower.step(input);
    assert(follower.state == line::Follower::SEARCH);
  }
}
