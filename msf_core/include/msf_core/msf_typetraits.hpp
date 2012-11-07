/*
 * mdf_typetraits.hpp
 *
 *  Created on: Nov 7, 2012
 *      Author: slynen
 */

#ifndef MDF_TYPETRAITS_HPP_
#define MDF_TYPETRAITS_HPP_

namespace msf_tmp{

//two types of same type
template <typename T, typename U> struct SameType{enum { value = false };};
template <typename T> struct SameType<T,T>{enum { value = true };};

//remove qualifiers
template<typename T>
struct StripReference{
	typedef T value;
};
template<typename T>
struct StripReference<T&>{
	typedef T value;
};
template<typename T>
struct StripReference<const T>{
	typedef const T value;
};
template<typename T>
struct StripReference<const T&>{
	typedef const T value;
};

template<typename T>
struct StripConstReference{
	typedef T value;
};
template<typename T>
struct StripConstReference<T&>{
	typedef T value;
};
template<typename T>
struct StripConstReference<const T&>{
	typedef T value;
};
template<typename T>
struct StripConstReference<const T>{
	typedef T value;
};

template<typename T>
struct StripConstPtr{
	typedef T value;
};
template<typename T>
struct StripConstPtr<T*>{
	typedef T value;
};
template<typename T>
struct StripConstPtr<const T*>{
	typedef T value;
};
template<typename T>
struct StripConstPtr<const T>{
	typedef T value;
};

//add qualifiers
template<typename T>
struct AddReference{
	typedef T& value;
};
template<typename T>
struct AddReference<T&>{
	typedef T& value;
};
template<typename T>
struct AddReference<const T>{
	typedef const T& value;
};
template<typename T>
struct AddReference<const T&>{
	typedef const T& value;
};

template<typename T>
struct AddConstReference{
	typedef const T& value;
};
template<typename T>
struct AddConstReference<T&>{
	typedef const T& value;
};
template<typename T>
struct AddConstReference<const T>{
	typedef const T& value;
};
template<typename T>
struct AddConstReference<const T&>{
	typedef const T& value;
};

template<typename T>
struct AddPtr{
	typedef T* value;
};
template<typename T>
struct AddPtr<T*>{
	typedef T* value;
};
template<typename T>
struct AddPtr<const T>{
	typedef const T* value;
};
template<typename T>
struct AddPtr<const T*>{
	typedef const T* value;
};

template<typename T>
struct AddConstPtr{
	typedef const T* value;
};
template<typename T>
struct AddConstPtr<T*>{
	typedef const T* value;
};
template<typename T>
struct AddConstPtr<const T>{
	typedef const T* value;
};
template<typename T>
struct AddConstPtr<const T*>{
	typedef const T* value;
};
}

#endif /* MDF_TYPETRAITS_HPP_ */
