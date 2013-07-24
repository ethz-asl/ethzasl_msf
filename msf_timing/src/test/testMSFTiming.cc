#include <chrono>
#include <iostream>
#include <thread>
#include <unistd.h>

#include "Timer.h"

int main(int argc, char** argv) {
  int elapsed_seconds = 0;

  for (int i = 0; i < 5; ++i) {
    msf::timing::Timer timer("test");

    std::chrono::time_point <std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();

    usleep(1e5);

    end = std::chrono::system_clock::now();

    elapsed_seconds += std::chrono::duration_cast <std::chrono::seconds>
    (end - start).count();
    timer.Stop();
  }
  msf::timing::Timing::Print(std::cout);
  msf::timing::Timing::Reset();
  msf::timing::Timing::Print(std::cout);
}
