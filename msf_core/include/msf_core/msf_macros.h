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
#ifndef MSF_MACROS_H_
#define MSF_MACROS_H_

#include <chrono>

#ifndef NUMERIC_PREC
#define NUMERIC_PREC 4 //number of decimal places
#endif

#ifdef UNUSEDPARAM
#elif defined(__GNUC__)
# define UNUSEDPARAM(x) UNUSED_ ## x __attribute__((unused))
#elif defined(__LCLINT__)
# define UNUSEDPARAM(x) /*@unused@*/ x
#else
# define UNUSEDPARAM(x) x
#endif

#ifndef UNUSED
# define UNUSED(x) (void)x;
#endif

#ifndef STREAMQUAT
#define STREAMQUAT(q) "["<<std::setprecision(3)<<(q).w()<<", "<<(q).x()<<", "<<(q).y()<<", "<<(q).z()<<std::setprecision(NUMERIC_PREC)<<"]"
#endif

#ifndef DEG2RAD
#define DEG2RAD (M_PI/180.0)
#endif

#ifdef WIN32
#define MSF_LIKELY(x)       (x)
#define MSF_UNLIKELY(x)     (x)
#else
#define MSF_LIKELY(x)       __builtin_expect((x),1)
#define MSF_UNLIKELY(x)     __builtin_expect((x),0)
#endif


#ifdef ROS_PACKAGE_NAME //use ROS if it is available
#include <ros/console.h>
#define MSF_INFO_STREAM(x) ROS_INFO_STREAM(x)
#define MSF_WARN_STREAM(x) ROS_WARN_STREAM(x)
#define MSF_ERROR_STREAM(x) ROS_ERROR_STREAM(x)

#define MSF_INFO_STREAM_ONCE(x) ROS_INFO_STREAM_ONCE(x)
#define MSF_WARN_STREAM_ONCE(x) ROS_WARN_STREAM_ONCE(x)
#define MSF_ERROR_STREAM_ONCE(x) ROS_ERROR_STREAM_ONCE(x)

#define MSF_LOG_STREAM_THROTTLE(rate, x) ROS_LOG_STREAM_THROTTLE(rate, x)
#define MSF_WARN_STREAM_THROTTLE(rate, x) ROS_WARN_STREAM_THROTTLE(rate, x)
#define MSF_ERROR_STREAM_THROTTLE(rate, x) ROS_ERROR_STREAM_THROTTLE(rate, x)

#define MSF_INFO_STREAM_COND(cond, x) ROS_INFO_STREAM_COND(cond, x)
#define MSF_WARN_STREAM_COND(cond, x) ROS_WARN_STREAM_COND(cond, x)
#define MSF_ERROR_STREAM_COND(cond, x) ROS_ERROR_STREAM_COND(cond, x)

#else

//adapted from rosconsole
//Copyright (c) 2008, Willow Garage, Inc.
#ifndef MSF_INFO_STREAM
#define MSF_INFO_STREAM(x) std::cerr<<"\033[0;0m[INFO] "<<x<<"\033[0;0m"<<std::endl;
#endif

#ifndef MSF_WARN_STREAM
#define MSF_WARN_STREAM(x) std::cerr<<"\033[0;33m[WARN] "<<x<<"\033[0;0m"<<std::endl;
#endif

#ifndef MSF_ERROR_STREAM
#define MSF_ERROR_STREAM(x) std::cerr<<"\033[1;31m[ERROR] "<<x<<"\033[0;0m"<<std::endl;
#endif

#define MSF_INFO_STREAM_ONCE(x) \
    do \
    { \
      static bool __log_stream_once__hit__ = false; \
      if (MSF_UNLIKELY(!__log_stream_once__hit__)) \
      { \
        __log_stream_once__hit__ = true; \
        MSF_INFO_STREAM(x); \
      } \
    } while(0)


#define MSF_WARN_STREAM_ONCE(x) \
    do \
    { \
      static bool __log_stream_once__hit__ = false; \
      if (MSF_UNLIKELY(!__log_stream_once__hit__)) \
      { \
        __log_stream_once__hit__ = true; \
        MSF_WARN_STREAM(x); \
      } \
    } while(0)


#define MSF_ERROR_STREAM_ONCE(x) \
    do \
    { \
      static bool __log_stream_once__hit__ = false; \
      if (MSF_UNLIKELY(!__log_stream_once__hit__)) \
      { \
        __log_stream_once__hit__ = true; \
        MSF_ERROR_STREAM(x); \
      } \
    } while(0)


#define MSF_LOG_STREAM_THROTTLE(rate, x) \
    do \
    { \
      static double __log_stream_throttle__last_hit__ = 0.0; \
      std::chrono::time_point<std::chrono::system_clock> __log_stream_throttle__now__ = std::chrono::system_clock::now(); \
      if (MSF_UNLIKELY(__log_stream_throttle__last_hit__ + rate <= std::chrono::duration_cast<std::chrono::seconds>(__log_stream_throttle__now__.time_since_epoch()).count())) \
      { \
        __log_stream_throttle__last_hit__ = std::chrono::duration_cast<std::chrono::seconds>(__log_stream_throttle__now__.time_since_epoch()).count(); \
        MSF_INFO_STREAM(x); \
      } \
    } while(0)

#define MSF_WARN_STREAM_THROTTLE(rate, x) \
    do \
    { \
      static double __log_stream_throttle__last_hit__ = 0.0; \
      std::chrono::time_point<std::chrono::system_clock> __log_stream_throttle__now__ = std::chrono::system_clock::now(); \
      if (MSF_UNLIKELY(__log_stream_throttle__last_hit__ + rate <= std::chrono::duration_cast<std::chrono::seconds>(__log_stream_throttle__now__.time_since_epoch()).count())) \
      { \
        __log_stream_throttle__last_hit__ = std::chrono::duration_cast<std::chrono::seconds>(__log_stream_throttle__now__.time_since_epoch()).count(); \
        MSF_WARN_STREAM(x); \
      } \
    } while(0)

#define MSF_ERROR_STREAM_THROTTLE(rate, x) \
    do \
    { \
      static double __log_stream_throttle__last_hit__ = 0.0; \
      std::chrono::time_point<std::chrono::system_clock> __log_stream_throttle__now__ = std::chrono::system_clock::now(); \
      if (MSF_UNLIKELY(__log_stream_throttle__last_hit__ + rate <= std::chrono::duration_cast<std::chrono::seconds>(__log_stream_throttle__now__.time_since_epoch()).count())) \
      { \
        __log_stream_throttle__last_hit__ = std::chrono::duration_cast<std::chrono::seconds>(__log_stream_throttle__now__.time_since_epoch()).count(); \
        MSF_ERROR_STREAM(x); \
      } \
    } while(0)

#define MSF_INFO_STREAM_COND(cond, x) \
    do \
    {  \
      if (MSF_UNLIKELY(cond)) \
      { \
        MSF_INFO_STREAM(x); \
      } \
    } while(0)

#define MSF_WARN_STREAM_COND(cond, x) \
    do \
    {  \
      if (MSF_UNLIKELY(cond)) \
      { \
        MSF_WARN_STREAM(x); \
      } \
    } while(0)

#define MSF_ERROR_STREAM_COND(cond, x) \
    do \
    {  \
      if (MSF_UNLIKELY(cond)) \
      { \
        MSF_ERROR_STREAM(x); \
      } \
    } while(0)

#endif
#endif /* MSF_MACROS_H_ */
