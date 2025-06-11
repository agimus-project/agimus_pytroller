#ifndef AGIMUS_PYTROLLER__VISIBILITY_HPP_
#define AGIMUS_PYTROLLER__VISIBILITY_HPP_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

// Define AGIMUS_PYTROLLER_[EXPORT, IMPORT, LOCAL]
// based on the OS
#if defined _WIN32 || defined __CYGWIN__

#ifdef __GNUC__
#define AGIMUS_PYTROLLER_EXPORT __attribute__((dllexport))
#define AGIMUS_PYTROLLER_IMPORT __attribute__((dllimport))
#else
#define AGIMUS_PYTROLLER_EXPORT __declspec(dllexport)
#define AGIMUS_PYTROLLER_IMPORT __declspec(dllimport)
#endif

// All symbols are hidden by default in windows
#define AGIMUS_PYTROLLER_LOCAL

#else // defined _WIN32 || defined __CYGWIN__

#if __GNUC__ >= 4
#define AGIMUS_PYTROLLER_EXPORT __attribute__((visibility("default")))
#define AGIMUS_PYTROLLER_IMPORT __attribute__((visibility("default")))
#define AGIMUS_PYTROLLER_LOCAL __attribute__((visibility("hidden")))
#else
#define AGIMUS_PYTROLLER_EXPORT
#define AGIMUS_PYTROLLER_IMPORT
#define AGIMUS_PYTROLLER_LOCAL
#endif

#endif // defined _WIN32 || defined __CYGWIN__

// Define AGIMUS_PYTROLLER_[PUBLIC, PRIVATE] based the following
// definitions forwarded by the build system:
// - AGIMUS_PYTROLLER_IS_SHARED (If the project is a shared lib)
// - AGIMUS_PYTROLLER_EXPORT (If we are building it directly)
#ifdef AGIMUS_PYTROLLER_IS_SHARED

// LFC lib is shared (.so)
#ifdef AGIMUS_PYTROLLER_DO_EXPORT
// We are building the shared lib -> EXPORT symbols
#define AGIMUS_PYTROLLER_PUBLIC AGIMUS_PYTROLLER_EXPORT
#else
// We are linking to the shared lib -> IMPORT symbols
#define AGIMUS_PYTROLLER_PUBLIC AGIMUS_PYTROLLER_IMPORT
#endif

#define AGIMUS_PYTROLLER_PRIVATE AGIMUS_PYTROLLER_LOCAL

#else // AGIMUS_PYTROLLER_IS_SHARED

// LFC lib is static (.a)
#define AGIMUS_PYTROLLER_PRIVATE
#define AGIMUS_PYTROLLER_PUBLIC

#endif

#endif // AGIMUS_PYTROLLER__VISIBILITY_HPP_
