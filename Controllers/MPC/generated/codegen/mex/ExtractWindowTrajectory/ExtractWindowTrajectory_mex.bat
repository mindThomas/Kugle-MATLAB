@echo off
set MATLAB=C:\PROGRA~1\MATLAB\R2018a
set MATLAB_ARCH=win64
set MATLAB_BIN="C:\Program Files\MATLAB\R2018a\bin"
set ENTRYPOINT=mexFunction
set OUTDIR=.\
set LIB_NAME=ExtractWindowTrajectory_mex
set MEX_NAME=ExtractWindowTrajectory_mex
set MEX_EXT=.mexw64
call setEnv.bat
echo # Make settings for ExtractWindowTrajectory > ExtractWindowTrajectory_mex.mki
echo COMPILER=%COMPILER%>> ExtractWindowTrajectory_mex.mki
echo COMPFLAGS=%COMPFLAGS%>> ExtractWindowTrajectory_mex.mki
echo OPTIMFLAGS=%OPTIMFLAGS%>> ExtractWindowTrajectory_mex.mki
echo DEBUGFLAGS=%DEBUGFLAGS%>> ExtractWindowTrajectory_mex.mki
echo LINKER=%LINKER%>> ExtractWindowTrajectory_mex.mki
echo LINKFLAGS=%LINKFLAGS%>> ExtractWindowTrajectory_mex.mki
echo LINKOPTIMFLAGS=%LINKOPTIMFLAGS%>> ExtractWindowTrajectory_mex.mki
echo LINKDEBUGFLAGS=%LINKDEBUGFLAGS%>> ExtractWindowTrajectory_mex.mki
echo MATLAB_ARCH=%MATLAB_ARCH%>> ExtractWindowTrajectory_mex.mki
echo OMPFLAGS= >> ExtractWindowTrajectory_mex.mki
echo OMPLINKFLAGS= >> ExtractWindowTrajectory_mex.mki
echo EMC_COMPILER=msvc150>> ExtractWindowTrajectory_mex.mki
echo EMC_CONFIG=optim>> ExtractWindowTrajectory_mex.mki
"C:\Program Files\MATLAB\R2018a\bin\win64\gmake" -j 1 -B -f ExtractWindowTrajectory_mex.mk
