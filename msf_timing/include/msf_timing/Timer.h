/*

 Copyright (c) 2013, Simon Lynen, ASL, ETH Zurich, Switzerland
 You can contact the author at <slynen at ethz dot ch>

 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of ETHZ-ASL nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL ETHZ-ASL BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 */


/* Adapted from Paul Furgale Schweizer Messer sm_timing*/

#ifndef MSF_TIMING_TIMER_H_
#define MSF_TIMING_TIMER_H_

#include <algorithm>
#include <chrono>
#include <limits>
#include <map>
#include <string>
#include <vector>

namespace msf_timing {

template<typename T, typename Total, int N>
class Accumulator {
 public:
  Accumulator()
      : window_samples_(0), totalsamples_(0),
        window_sum_(0), sum_(0),
        min_(std::numeric_limits <T> ::max()),
        max_(std::numeric_limits <T> ::min()) { }

  void Add(T sample) {
    if (window_samples_ < N) {
      samples_[window_samples_++] = sample;
      window_sum_ += sample;
    } else {
      T& oldest = samples_[window_samples_++ % N];
      window_sum_ += sample - oldest;
      oldest = sample;
    }
    sum_ += sample;
    ++totalsamples_;
    if (sample > max_) {
      max_ = sample;
    }
    if (sample < min_) {
      min_ = sample;
    }
  }

  int TotalSamples() const {
    return totalsamples_;
  }

  double Sum() const {
    return sum_;
  }

  double Mean() const {
    return sum_ / totalsamples_;
  }

  double RollingMean() const {
    return window_sum_ / std::min(window_samples_, N);
  }

  double Max() const {
    return max_;
  }

  double Min() const {
    return min_;
  }

  double LazyVariance() const {
    if(window_samples_ == 0){
      return 0.0;
    }
    double var = 0;
    double mean = RollingMean();
    for (int i = 0; i < std::min(window_samples_, N); ++i) {
      var += (samples_[i] - mean) * (samples_[i] - mean);
    }
    var /= std::min(window_samples_, N);
    return var;
  }

 private:
  int window_samples_;
  int totalsamples_;
  Total window_sum_;
  Total sum_;
  T min_;
  T max_;
  T samples_[N];
};

struct TimerMapValue {
  TimerMapValue() { }

  //Create an accumulator with specified window size.
  Accumulator<double, double, 50> acc_;
};

// A class that has the timer interface but does nothing. Swapping this in in
// place of the Timer class (say with a typedef) should allow one to disable
// timing. Because all of the functions are inline, they should just disappear.
class DummyTimer {
 public:
  DummyTimer(size_t handle, bool constructStopped = false) { }
  DummyTimer(std::string const& tag, bool constructStopped = false) { }
  ~DummyTimer() { }

  void Start() { }
  void Stop() { }
  bool IsTiming() {
    return false;
  }
};

class Timer {
 public:
  Timer(size_t handle, bool constructStopped = false);
  Timer(std::string const& tag, bool constructStopped = false);
  ~Timer();

  void Start();
  void Stop();
  bool IsTiming() const;
 private:
  std::chrono::time_point<std::chrono::system_clock> time_;

  bool timing_;
  size_t handle_;
};

class Timing {
 public:
  typedef std::map<std::string, size_t> map_t;
  friend class Timer;
  // Definition of static functions to query the timers.
  static size_t GetHandle(std::string const& tag);
  static std::string GetTag(size_t handle);
  static double GetTotalSeconds(size_t handle);
  static double GetTotalSeconds(std::string const& tag);
  static double GetMeanSeconds(size_t handle);
  static double GetMeanSeconds(std::string const& tag);
  static size_t GetNumSamples(size_t handle);
  static size_t GetNumSamples(std::string const& tag);
  static double GetVarianceSeconds(size_t handle);
  static double GetVarianceSeconds(std::string const& tag);
  static double GetMinSeconds(size_t handle);
  static double GetMinSeconds(std::string const& tag);
  static double GetMaxSeconds(size_t handle);
  static double GetMaxSeconds(std::string const& tag);
  static double GetHz(size_t handle);
  static double GetHz(std::string const& tag);
  static void Print(std::ostream& out);
  static std::string Print();
  static std::string SecondsToTimeString(double seconds);
  static void Reset();
  static const map_t& GetTimers() {return Instance().tagMap_;}
 private:
  void AddTime(size_t handle, double seconds);

  static Timing& Instance();

  Timing();
  ~Timing();

  typedef std::vector<TimerMapValue> list_t;

  list_t timers_;
  map_t tagMap_;
  size_t maxTagLength_;
};

#if ENABLE_MSF_TIMING
typedef Timer DebugTimer;
#else
typedef DummyTimer DebugTimer;
#endif

}  // namespace msf_timing
#endif  // MSF_TIMING_TIMER_H_
