/*
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
	typedef T result_t;
};
template<typename T>
struct StripReference<T&>{
	typedef T result_t;
};
template<typename T>
struct StripReference<const T>{
	typedef const T result_t;
};
template<typename T>
struct StripReference<const T&>{
	typedef const T result_t;
};

template<typename T>
struct StripConstReference{
	typedef T result_t;
};
template<typename T>
struct StripConstReference<T&>{
	typedef T result_t;
};
template<typename T>
struct StripConstReference<const T&>{
	typedef T result_t;
};
template<typename T>
struct StripConstReference<const T>{
	typedef T result_t;
};

template<typename T>
struct StripConstPtr{
	typedef T result_t;
};
template<typename T>
struct StripConstPtr<T*>{
	typedef T result_t;
};
template<typename T>
struct StripConstPtr<const T*>{
	typedef T result_t;
};
template<typename T>
struct StripConstPtr<const T>{
	typedef T result_t;
};

//add qualifiers
template<typename T>
struct AddReference{
	typedef T& result_t;
};
template<typename T>
struct AddReference<T&>{
	typedef T& result_t;
};
template<typename T>
struct AddReference<const T>{
	typedef const T& result_t;
};
template<typename T>
struct AddReference<const T&>{
	typedef const T& result_t;
};

template<typename T>
struct AddConstReference{
	typedef const T& result_t;
};
template<typename T>
struct AddConstReference<T&>{
	typedef const T& result_t;
};
template<typename T>
struct AddConstReference<const T>{
	typedef const T& result_t;
};
template<typename T>
struct AddConstReference<const T&>{
	typedef const T& result_t;
};

template<typename T>
struct AddPtr{
	typedef T* result_t;
};
template<typename T>
struct AddPtr<T*>{
	typedef T* result_t;
};
template<typename T>
struct AddPtr<const T>{
	typedef const T* result_t;
};
template<typename T>
struct AddPtr<const T*>{
	typedef const T* result_t;
};

template<typename T>
struct AddConstPtr{
	typedef const T* result_t;
};
template<typename T>
struct AddConstPtr<T*>{
	typedef const T* result_t;
};
template<typename T>
struct AddConstPtr<const T>{
	typedef const T* result_t;
};
template<typename T>
struct AddConstPtr<const T*>{
	typedef const T* result_t;
};

template<typename T>
struct IsReferenceType{
	enum{
		value = false
	};
};
template<typename T>
struct IsReferenceType<T&>{
	enum{
		value = true
	};
};
template<typename T>
struct IsReferenceType<const T&>{
	enum{
		value = true
	};
};


template<typename T>
struct IsPointerType{
	enum{
		value = false
	};
};
template<typename T>
struct IsPointerType<T*>{
	enum{
		value = true
	};
};
template<typename T>
struct IsPointerType<const T*>{
	enum{
		value = true
	};
};

}

#endif /* MDF_TYPETRAITS_HPP_ */
