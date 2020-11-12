//
// MATLAB Compiler: 7.1 (R2019b)
// Date: Mon Jun 22 10:29:05 2020
// Arguments:
// "-B""macro_default""-W""cpplib:libQUADASSINPROG""-T""link:lib""QUADASSINPROG.
// m"
//

#ifndef libQUADASSINPROG_h
#define libQUADASSINPROG_h 1

#if defined(__cplusplus) && !defined(mclmcrrt_h) && defined(__linux__)
#  pragma implementation "mclmcrrt.h"
#endif
#include "mclmcrrt.h"
#include "mclcppclass.h"
#ifdef __cplusplus
extern "C" { // sbcheck:ok:extern_c
#endif

/* This symbol is defined in shared libraries. Define it here
 * (to nothing) in case this isn't a shared library. 
 */
#ifndef LIB_libQUADASSINPROG_C_API 
#define LIB_libQUADASSINPROG_C_API /* No special import/export declaration */
#endif

/* GENERAL LIBRARY FUNCTIONS -- START */

extern LIB_libQUADASSINPROG_C_API 
bool MW_CALL_CONV libQUADASSINPROGInitializeWithHandlers(
       mclOutputHandlerFcn error_handler, 
       mclOutputHandlerFcn print_handler);

extern LIB_libQUADASSINPROG_C_API 
bool MW_CALL_CONV libQUADASSINPROGInitialize(void);

extern LIB_libQUADASSINPROG_C_API 
void MW_CALL_CONV libQUADASSINPROGTerminate(void);

extern LIB_libQUADASSINPROG_C_API 
void MW_CALL_CONV libQUADASSINPROGPrintStackTrace(void);

/* GENERAL LIBRARY FUNCTIONS -- END */

/* C INTERFACE -- MLX WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- START */

extern LIB_libQUADASSINPROG_C_API 
bool MW_CALL_CONV mlxQUADASSINPROG(int nlhs, mxArray *plhs[], int nrhs, mxArray *prhs[]);

/* C INTERFACE -- MLX WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- END */

#ifdef __cplusplus
}
#endif


/* C++ INTERFACE -- WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- START */

#ifdef __cplusplus

/* On Windows, use __declspec to control the exported API */
#if defined(_MSC_VER) || defined(__MINGW64__)

#ifdef EXPORTING_libQUADASSINPROG
#define PUBLIC_libQUADASSINPROG_CPP_API __declspec(dllexport)
#else
#define PUBLIC_libQUADASSINPROG_CPP_API __declspec(dllimport)
#endif

#define LIB_libQUADASSINPROG_CPP_API PUBLIC_libQUADASSINPROG_CPP_API

#else

#if !defined(LIB_libQUADASSINPROG_CPP_API)
#if defined(LIB_libQUADASSINPROG_C_API)
#define LIB_libQUADASSINPROG_CPP_API LIB_libQUADASSINPROG_C_API
#else
#define LIB_libQUADASSINPROG_CPP_API /* empty! */ 
#endif
#endif

#endif

extern LIB_libQUADASSINPROG_CPP_API void MW_CALL_CONV QUADASSINPROG(int nargout, mwArray& xx, mwArray& ffval, mwArray& eexitflag, const mwArray& H, const mwArray& f, const mwArray& A, const mwArray& b, const mwArray& Aeq, const mwArray& beq, const mwArray& num_iters);

/* C++ INTERFACE -- WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- END */
#endif

#endif
