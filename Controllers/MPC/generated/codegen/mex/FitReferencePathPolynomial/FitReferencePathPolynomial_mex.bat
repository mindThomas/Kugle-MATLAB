@echo off
set MATLAB=C:\PROGRA~1\MATLAB\R2018a
set MATLAB_ARCH=win64
set MATLAB_BIN="C:\Program Files\MATLAB\R2018a\bin"
set ENTRYPOINT=mexFunction
set OUTDIR=.\
set LIB_NAME=FitReferencePathPolynomial_mex
set MEX_NAME=FitReferencePathPolynomial_mex
set MEX_EXT=.mexw64
call setEnv.bat
echo # Make settings for FitReferencePathPolynomial > FitReferencePathPolynomial_mex.mki
echo COMPILER=%COMPILER%>> FitReferencePathPolynomial_mex.mki
echo COMPFLAGS=%COMPFLAGS%>> FitReferencePathPolynomial_mex.mki
echo OPTIMFLAGS=%OPTIMFLAGS%>> FitReferencePathPolynomial_mex.mki
echo DEBUGFLAGS=%DEBUGFLAGS%>> FitReferencePathPolynomial_mex.mki
echo LINKER=%LINKER%>> FitReferencePathPolynomial_mex.mki
echo LINKFLAGS=%LINKFLAGS%>> FitReferencePathPolynomial_mex.mki
echo LINKOPTIMFLAGS=%LINKOPTIMFLAGS%>> FitReferencePathPolynomial_mex.mki
echo LINKDEBUGFLAGS=%LINKDEBUGFLAGS%>> FitReferencePathPolynomial_mex.mki
echo MATLAB_ARCH=%MATLAB_ARCH%>> FitReferencePathPolynomial_mex.mki
echo OMPFLAGS= >> FitReferencePathPolynomial_mex.mki
echo OMPLINKFLAGS= >> FitReferencePathPolynomial_mex.mki
echo EMC_COMPILER=msvc150>> FitReferencePathPolynomial_mex.mki
echo EMC_CONFIG=optim>> FitReferencePathPolynomial_mex.mki
"C:\Program Files\MATLAB\R2018a\bin\win64\gmake" -j 1 -B -f FitReferencePathPolynomial_mex.mk
