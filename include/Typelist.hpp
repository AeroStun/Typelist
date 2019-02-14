#pragma once
#include <cstddef>
#include <type_traits>

template <class...>
struct TypeList;

namespace tl {

  template <typename T>
  struct Front;

  template <typename T, typename... Ts>
  struct Front<TypeList<T, Ts...>> {
    using type = T;
  };

  template <typename T>
  using Front_t = typename Front<T>::type;

  template <std::size_t, typename>
  struct At;

  template <std::size_t N, typename... Ts>
  struct At<N, TypeList<Ts...>> {
  private:
    template <std::size_t, class>
    struct impl;

    template <std::size_t I, class Head, class... Tail>
    struct impl<I, TypeList<Head, Tail...>> : impl<I - 1, TypeList<Tail...>> { };

    template <class Head, class... Tail>
    struct impl<0, TypeList<Head, Tail...>> {
      typedef Head type;
    };

  public:
    using type = impl<N, Ts...>;
  };

  template <std::size_t N, typename T>
  using At_t = typename At<N, T>::type;

  template <typename, typename>
  struct FindFirst;

  template <typename T, typename... Ts>
  struct FindFirst<T, TypeList<Ts...>> {
    constexpr static std::size_t value = [](){
      auto idx = 0;
      bool found = false;
      ([&](){
        if(!found)
          ++idx;
        if(std::is_same_v<T, Ts>)
          found = true;
      }(), ...);
      return idx < sizeof...(Ts) ? idx : -1;
    }();
  };

  template<typename T, typename U>
  constexpr auto FindFirst_v = FindFirst<T, U>::value;

  template <typename, typename>
  struct FindLast;

  template <typename T, typename... Ts>
  struct FindLast<T, TypeList<Ts...>> {
    constexpr static std::size_t value = [](){
      auto idx = 0;
      auto last_match = -1;
      ([&](){
        if(std::is_same_v<T, Ts>)
          last_match = idx;
        ++idx;
      }(), ...);
      return last_match;
    }();
  };

  template<typename T, typename U>
  constexpr auto FindLast_v = FindLast<T, U>::value;

  template<typename, typename>
  struct PushFront;

  template<typename T, typename... Args>
  struct PushFront<T, TypeList<Args...>> {
    using type = TypeList<T, Args...>;
  };

  template<typename T, typename U>
  using PushFront_t = typename PushFront<T, U>::type;

  template<typename, typename>
  struct PushBack;

  template<typename T, typename... Args>
  struct PushBack<T, TypeList< Args...>> {
    using type = TypeList<Args..., T>;
  };

  template<typename T, typename U>
  using PushBack_t = typename PushBack<T, U>::type;

  template<typename>
  struct PopFront;

  template<typename T, typename... Args>
  struct PopFront<TypeList<T, Args...>> {
    using type = TypeList<Args...>;
  };

  template <>
  struct PopFront<TypeList<>> {
    using type = TypeList<>;
  };

  template<typename T>
  using PopFront_t = typename PopFront<T>::type;

  template<typename>
  struct PopBack;

  template<typename... Args>
  struct PopBack<TypeList<Args...>> {
  private:
    template <std::size_t, typename U, typename V>
    struct impl;

    template <std::size_t I, typename U, typename... Vs>
    struct impl<I, U, TypeList<Vs...>> {
      using next = impl<I - 1, PopFront_t<U>, TypeList<Vs..., Front_t<U>>>;
    };
    template <typename U, typename V>
    struct impl<0, TypeList<U>, V> {
      using next = V;
    };
  public:
    using type = typename impl<sizeof...(Args) - 1, TypeList<Args...>, TypeList<>>::next;
  };

  template <>
  struct PopBack<TypeList<>> {
    using type = TypeList<>;
  };

  template<typename T>
  using PopBack_t = typename PopBack<T>::type;

  template<template<typename> class P, typename... Ts>
  struct Filter;

  template<template<typename> class P>
  struct Filter<P> {
    using type = TypeList<>;
  };

  template<template<typename> class P, typename Head, typename... Tail>
  struct Filter<P, Head, Tail...> {
    using type = typename std::conditional_t<P<Head>::value,
        typename PushFront<Head, typename Filter<P, Tail...>::type>::type,
        typename Filter<P, Tail...>::type>;
  };

  template<template<typename> class P, typename... Ts>
  using Filter_t = typename Filter<P, Ts...>::type;

  template<template<typename> class P, typename... Ts>
  struct Filter_tuple {
    using type = typename Filter_t<P, Ts...>::type;
  };

  template<template<typename> class P>
  struct Filter_tuple<P> {
    using type = TypeList<>;
  };

  template<template<typename> class P, typename... Ts>
  using Filter_tuple_t = typename Filter_tuple<P, Ts...>::type;

  template<template<typename, std::size_t> class P, std::size_t N, typename... Ts>
  struct Transform;

  template<template<typename, std::size_t> class P, std::size_t N>
  struct Transform<P, N> {
    using type = TypeList<>;
  };

  template<template<typename, std::size_t> class P, std::size_t N, typename Head, typename... Tail>
  struct Transform<P, N, Head, Tail...> {
    using type = PushFront_t<typename P<Head, N>::type, typename Transform<P, N + 1, Tail...>::type>;
  };

  template<template<typename, std::size_t> class P, std::size_t N, typename... Ts>
  using Transform_t = typename Transform<P, N, Ts...>::type;

  template<template<typename, std::size_t> class P, typename T>
  struct Transform_tuple;

  template<template<typename, std::size_t> class P>
  struct Transform_tuple<P, TypeList<>> {
    using type = Transform<P, 0>;
  };

  template<template<typename, std::size_t> class P, typename... Ts>
  struct Transform_tuple<P, TypeList < Ts...>> {
    using type = Transform_t<P, 0, Ts...>;
  };

  template<template<typename, std::size_t> class P, typename T>
  using Transform_tuple_t = typename Transform_tuple<P, T>::type;
}