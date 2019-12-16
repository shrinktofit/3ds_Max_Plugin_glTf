
#pragma once

#if APRICOT_EXPORTS_API
#define APRICOT_API __declspec(dllexport)
#else
#define APRICOT_API __declspec(dllimport)
#endif