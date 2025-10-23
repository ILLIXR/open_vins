#pragma once
#if defined(_WIN32) || defined(_WIN64)
    #ifdef BUILDING_OV_CORE
        #define OV_CORE_API_EXPORT __declspec(dllexport)
    #else
        #define OV_CORE_API_EXPORT __declspec(dllimport)
    #endif
    #ifdef BUILDING_OV_MSCKF
        #define OV_MSCKF_API_EXPORT __declspec(dllexport)
    #else
        #define OV_MSCKF_API_EXPORT __declspec(dllimport)
    #endif
#else
#define OV_CORE_API_EXPORT __attribute__((visibility("default")))
#define OV_MSCKF_API_EXPORT __attribute__((visibility("default")))
#endif
