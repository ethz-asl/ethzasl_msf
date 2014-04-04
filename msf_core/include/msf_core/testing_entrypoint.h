/*
 * Copyright (C) 2014 Simon Lynen, ASL, ETH Zurich, Switzerland
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

#ifndef TESTING_ENTRYPOINT_H_
#define TESTING_ENTRYPOINT_H_

#include <gtest/gtest.h>
#include <glog/logging.h>

// Let the Eclipse parser see the macro.
#ifndef TEST
#define TEST(a, b) int Test_##a##_##b()
#endif

#define MSF_UNITTEST_ENTRYPOINT\
  int main(int argc, char** argv) {\
 ::testing::InitGoogleTest(&argc, argv);\
  google::InitGoogleLogging(argv[0]);\
  return RUN_ALL_TESTS();\
}

#endif   // TESTING_ENTRYPOINT_H_
