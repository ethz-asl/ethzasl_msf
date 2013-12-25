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
#ifndef MSF_TYPETRAITS_H_
#define MSF_TYPETRAITS_H_

namespace msf_tmp {
// Two types of same type.
template<typename T, typename U> struct SameType {
  enum {
    value = false
  };
};
template<typename T> struct SameType<T, T> {
  enum {
    value = true
  };
};

// Strip qualifiers.
template<typename T>
struct StripReference {
  typedef T result_t;
};
template<typename T>
struct StripReference<T&> {
  typedef T result_t;
};
template<typename T>
struct StripReference<const T> {
  typedef const T result_t;
};
template<typename T>
struct StripReference<const T&> {
  typedef const T result_t;
};

template<typename T>
struct StripConstReference {
  typedef T result_t;
};
template<typename T>
struct StripConstReference<T&> {
  typedef T result_t;
};
template<typename T>
struct StripConstReference<const T&> {
  typedef T result_t;
};
template<typename T>
struct StripConstReference<const T> {
  typedef T result_t;
};

template<typename T>
struct StripConstPtr {
  typedef T result_t;
};
template<typename T>
struct StripConstPtr<T*> {
  typedef T result_t;
};
template<typename T>
struct StripConstPtr<const T*> {
  typedef T result_t;
};
template<typename T>
struct StripConstPtr<const T> {
  typedef T result_t;
};

// Add qualifiers.
template<typename T>
struct AddReference {
  typedef T& result_t;
};
template<typename T>
struct AddReference<T&> {
  typedef T& result_t;
};
template<typename T>
struct AddReference<const T> {
  typedef const T& result_t;
};
template<typename T>
struct AddReference<const T&> {
  typedef const T& result_t;
};

template<typename T>
struct AddConstReference {
  typedef const T& result_t;
};
template<typename T>
struct AddConstReference<T&> {
  typedef const T& result_t;
};
template<typename T>
struct AddConstReference<const T> {
  typedef const T& result_t;
};
template<typename T>
struct AddConstReference<const T&> {
  typedef const T& result_t;
};

template<typename T>
struct AddPtr {
  typedef T* result_t;
};
template<typename T>
struct AddPtr<T*> {
  typedef T* result_t;
};
template<typename T>
struct AddPtr<const T> {
  typedef const T* result_t;
};
template<typename T>
struct AddPtr<const T*> {
  typedef const T* result_t;
};

template<typename T>
struct AddConstPtr {
  typedef const T* result_t;
};
template<typename T>
struct AddConstPtr<T*> {
  typedef const T* result_t;
};
template<typename T>
struct AddConstPtr<const T> {
  typedef const T* result_t;
};
template<typename T>
struct AddConstPtr<const T*> {
  typedef const T* result_t;
};

template<typename T>
struct IsReferenceType {
  enum {
    value = false
  };
};
template<typename T>
struct IsReferenceType<T&> {
  enum {
    value = true
  };
};
template<typename T>
struct IsReferenceType<const T&> {
  enum {
    value = true
  };
};

template<typename T>
struct IsPointerType {
  enum {
    value = false
  };
};
template<typename T>
struct IsPointerType<T*> {
  enum {
    value = true
  };
};
template<typename T>
struct IsPointerType<const T*> {
  enum {
    value = true
  };
};

}  // namespace msf_tmp
#endif  // MSF_TYPETRAITS_H_
