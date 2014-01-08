/*
 * Copyright (C) 2012-2013 Simon Lynen, ASL, ETH Zurich, Switzerland
 * You can contact the author at <slynen at ethz dot ch>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <chrono>
#include <gtest/gtest.h>
#include <iostream>
#include <thread>
#include <unistd.h>

#include <msf_timing/Timer.h>

namespace {
TEST(timing, timing) {
  int elapsed_seconds = 0;

  for (int i = 0; i < 5; ++i) {
    msf_timing::Timer timer("test");

    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();

    usleep(1e5);

    end = std::chrono::system_clock::now();

    elapsed_seconds += std::chrono::duration_cast < std::chrono::seconds
        > (end - start).count();
    timer.Stop();
  }
  msf_timing::Timing::Print(std::cout);
  msf_timing::Timing::Reset();
  msf_timing::Timing::Print(std::cout);
}
}  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
