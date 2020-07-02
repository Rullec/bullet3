//
// MATLAB Compiler: 7.1 (R2019b)
// Date: Mon Jun 22 10:29:05 2020
// Arguments:
// "-B""macro_default""-W""cpplib:libQUADASSINPROG""-T""link:lib""QUADASSINPROG.
// m"
//

#define EXPORTING_libQUADASSINPROG 1
#include "libQUADASSINPROG.h"

static HMCRINSTANCE _mcr_inst = NULL; /* don't use nullptr; this may be either C or C++ */

#ifdef __cplusplus
extern "C" { // sbcheck:ok:extern_c
#endif

static int mclDefaultPrintHandler(const char *s)
{
    return mclWrite(1 /* stdout */, s, sizeof(char)*strlen(s));
}

#ifdef __cplusplus
} /* End extern C block */
#endif

#ifdef __cplusplus
extern "C" { // sbcheck:ok:extern_c
#endif

static int mclDefaultErrorHandler(const char *s)
{
    int written = 0;
    size_t len = 0;
    len = strlen(s);
    written = mclWrite(2 /* stderr */, s, sizeof(char)*len);
    if (len > 0 && s[ len-1 ] != '\n')
        written += mclWrite(2 /* stderr */, "\n", sizeof(char));
    return written;
}

#ifdef __cplusplus
} /* End extern C block */
#endif

/* This symbol is defined in shared libraries. Define it here
 * (to nothing) in case this isn't a shared library. 
 */
#ifndef LIB_libQUADASSINPROG_C_API
#define LIB_libQUADASSINPROG_C_API /* No special import/export declaration */
#endif

LIB_libQUADASSINPROG_C_API 
bool MW_CALL_CONV libQUADASSINPROGInitializeWithHandlers(
    mclOutputHandlerFcn error_handler,
    mclOutputHandlerFcn print_handler)
{
    int bResult = 0;
    if (_mcr_inst)
        return true;
    if (!mclmcrInitialize())
        return false;
    {
        mclCtfStream ctfStream = 
            mclGetEmbeddedCtfStream((void *)(libQUADASSINPROGInitializeWithHandlers));
        if (ctfStream) {
            bResult = mclInitializeComponentInstanceEmbedded(&_mcr_inst,
                                                             error_handler, 
                                                             print_handler,
                                                             ctfStream);
            mclDestroyStream(ctfStream);
        } else {
            bResult = 0;
        }
    }  
    if (!bResult)
    return false;
    return true;
}

LIB_libQUADASSINPROG_C_API 
bool MW_CALL_CONV libQUADASSINPROGInitialize(void)
{
    return libQUADASSINPROGInitializeWithHandlers(mclDefaultErrorHandler, 
                                                mclDefaultPrintHandler);
}

LIB_libQUADASSINPROG_C_API 
void MW_CALL_CONV libQUADASSINPROGTerminate(void)
{
    if (_mcr_inst)
        mclTerminateInstance(&_mcr_inst);
}

LIB_libQUADASSINPROG_C_API 
void MW_CALL_CONV libQUADASSINPROGPrintStackTrace(void) 
{
    char** stackTrace;
    int stackDepth = mclGetStackTrace(&stackTrace);
    int i;
    for(i=0; i<stackDepth; i++)
    {
        mclWrite(2 /* stderr */, stackTrace[i], sizeof(char)*strlen(stackTrace[i]));
        mclWrite(2 /* stderr */, "\n", sizeof(char)*strlen("\n"));
    }
    mclFreeStackTrace(&stackTrace, stackDepth);
}


LIB_libQUADASSINPROG_C_API 
bool MW_CALL_CONV mlxQUADASSINPROG(int nlhs, mxArray *plhs[], int nrhs, mxArray *prhs[])
{
    return mclFeval(_mcr_inst, "QUADASSINPROG", nlhs, plhs, nrhs, prhs);
}

LIB_libQUADASSINPROG_CPP_API 
void MW_CALL_CONV QUADASSINPROG(int nargout, mwArray& xx, mwArray& ffval, mwArray& 
                                eexitflag, const mwArray& H, const mwArray& f, const 
                                mwArray& A, const mwArray& b, const mwArray& Aeq, const 
                                mwArray& beq, const mwArray& num_iters)
{
    mclcppMlfFeval(_mcr_inst, "QUADASSINPROG", nargout, 3, 7, &xx, &ffval, &eexitflag, &H, &f, &A, &b, &Aeq, &beq, &num_iters);
}

