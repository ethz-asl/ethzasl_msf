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

#ifndef MSF_GLOG_H_
#define MSF_GLOG_H_

#ifdef MSF_HAVE_GLOG
#include <glog/logging.h>
#else
#include <cassert>
#include <iosfwd>
#include <iostream>

#define CAST_TO_USED(x) static_cast<void>(x)

#define CHECK_NOTNULL(x) assert(x != nullptr); CAST_TO_USED(x);
#define CHECK_EQ(x, y) assert(x == y); CAST_TO_USED(x); CAST_TO_USED(y); \
    std::cout << ""
#define CHECK_NE(x, y) assert(x != y); CAST_TO_USED(x); CAST_TO_USED(y); \
    std::cout << ""
#define CHECK_GT(x, y) assert(x > y); CAST_TO_USED(x); CAST_TO_USED(y); \
    std::cout << ""
#define CHECK_LT(x, y) assert(x < y); CAST_TO_USED(x); CAST_TO_USED(y); \
    std::cout << ""
#define CHECK_GE(x, y) assert(x >= y); CAST_TO_USED(x); CAST_TO_USED(y); \
    std::cout << ""
#define CHECK_LE(x, y) assert(x <= y); CAST_TO_USED(x); CAST_TO_USED(y); \
    std::cout << ""
#define CHECK(x) assert(x); CAST_TO_USED(x); \
    std::cout << ""
#define LOG(WARNING) std ::cout << ""

namespace google {
void InitGoogleLogging(const std::string&) { };
}  // namespace google

#endif  // MSF_HAVE_GLOG
#endif  // MSF_GLOG_H_
