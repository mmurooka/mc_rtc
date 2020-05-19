#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define MyFirstFsmController_DLLIMPORT __declspec(dllimport)
#  define MyFirstFsmController_DLLEXPORT __declspec(dllexport)
#  define MyFirstFsmController_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define MyFirstFsmController_DLLIMPORT __attribute__((visibility("default")))
#    define MyFirstFsmController_DLLEXPORT __attribute__((visibility("default")))
#    define MyFirstFsmController_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define MyFirstFsmController_DLLIMPORT
#    define MyFirstFsmController_DLLEXPORT
#    define MyFirstFsmController_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef MyFirstFsmController_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define MyFirstFsmController_DLLAPI
#  define MyFirstFsmController_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef MyFirstFsmController_EXPORTS
#    define MyFirstFsmController_DLLAPI MyFirstFsmController_DLLEXPORT
#  else
#    define MyFirstFsmController_DLLAPI MyFirstFsmController_DLLIMPORT
#  endif // MyFirstFsmController_EXPORTS
#  define MyFirstFsmController_LOCAL MyFirstFsmController_DLLLOCAL
#endif // MyFirstFsmController_STATIC