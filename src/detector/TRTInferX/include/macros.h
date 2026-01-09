#ifndef __MACROS_H
#define __MACROS_H

#ifdef API_EXPORTS
#if defined(_MSC_VER)
#define API __declspec(dllexport)
#else
#define API __attribute__((visibility("default")))
#endif
#else

#if defined(_MSC_VER)
#define API __declspec(dllimport)
#else
#define API
#endif
#endif  // API_EXPORTS

#include <type_traits>

#if NV_TENSORRT_MAJOR >= 8
#define TRT_NOEXCEPT noexcept
#define TRT_CONST_ENQUEUE const
#else
#define TRT_NOEXCEPT
#define TRT_CONST_ENQUEUE
#endif

template <typename T, typename = void>
struct trt_has_destroy : std::false_type
{
};

template <typename T>
struct trt_has_destroy<T, std::void_t<decltype(std::declval<T &>().destroy())>> : std::true_type
{
};

template <typename T>
inline void trt_destroy(T *obj)
{
    if (!obj)
        return;
    if constexpr (trt_has_destroy<T>::value)
        obj->destroy();
    else
        delete obj;
}

#endif  // __MACROS_H
