#ifndef ROBOTNAME_NAVIGATION__VISIBILITY_CONTROL_H_
#define ROBOTNAME_NAVIGATION__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROBOTNAME_NAVIGATION_EXPORT __attribute__ ((dllexport))
    #define ROBOTNAME_NAVIGATION_IMPORT __attribute__ ((dllimport))
  #else
    #define ROBOTNAME_NAVIGATION_EXPORT __declspec(dllexport)
    #define ROBOTNAME_NAVIGATION_IMPORT __declspec(dllimport)
  #endif
  #ifdef ROBOTNAME_NAVIGATION_BUILDING_DLL
    #define ROBOTNAME_NAVIGATION_PUBLIC ROBOTNAME_NAVIGATION_EXPORT
  #else
    #define ROBOTNAME_NAVIGATION_PUBLIC ROBOTNAME_NAVIGATION_IMPORT
  #endif
  #define ROBOTNAME_NAVIGATION_PUBLIC_TYPE ROBOTNAME_NAVIGATION_PUBLIC
  #define ROBOTNAME_NAVIGATION_LOCAL
#else
  #define ROBOTNAME_NAVIGATION_EXPORT __attribute__ ((visibility("default")))
  #define ROBOTNAME_NAVIGATION_IMPORT
  #if __GNUC__ >= 4
    #define ROBOTNAME_NAVIGATION_PUBLIC __attribute__ ((visibility("default")))
    #define ROBOTNAME_NAVIGATION_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ROBOTNAME_NAVIGATION_PUBLIC
    #define ROBOTNAME_NAVIGATION_LOCAL
  #endif
  #define ROBOTNAME_NAVIGATION_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // ROBOTNAME_NAVIGATION__VISIBILITY_CONTROL_H_