/*
 * Copyright (c) 2012, Markus Achtelik, ASL, ETH Zurich, Switzerland
 * You can contact the author at <acmarkus at ethz dot ch>
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
#ifndef SINCOS_H_
#define SINCOS_H_

#ifdef __APPLE__
/* Possible method for setting __x87_inline_math__ */
#if (defined(__i386__) || defined(i386) || defined(__amd64) || defined(__x86_64))
#if (!defined(__x87_inline_math__) && defined(__FAST_MATH__))
#define __x87_inline_math__
#endif
#endif

#ifdef __x87_inline_math__
/*
 ** Compute sine and cosine at same time (faster than separate calls).
 ** (*s) gets sin(x)
 ** (*c) gets cos(x)
 */
#define sincos(x,s,c) sincos_x87_inline(x,s,c)
void sincos_x87_inline(double x,double *s,double *c);
extern __inline__ void sincos_x87_inline(double x,double *s,double *c)
{
  __asm__ ("fsincos;" : "=t" (*c), "=u" (*s) : "0" (x) : "st(7)");
}
#else
#define sincos(th,x,y) { (*(x))=sin(th); (*(y))=cos(th); }
#endif
#endif

#endif  // SINCOS_H_
