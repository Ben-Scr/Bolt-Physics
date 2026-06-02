#pragma once

#if (defined(_WIN32) || defined(_WIN64))
#ifdef INDEX_PHYS_BUILD_DLL
#define INDEX_PHYS_API __declspec(dllexport)
#elif INDEX_PHYS_IMPORT_DLL
#define INDEX_PHYS_API __declspec(dllimport)
#else
#define INDEX_PHYS_API
#endif
#else
#define INDEX_PHYS_API
#endif
