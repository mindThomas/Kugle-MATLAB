@echo off
set MATLAB=C:\PROGRA~1\MATLAB\R2018a
set MATLAB_ARCH=win64
set MATLAB_BIN="C:\Program Files\MATLAB\R2018a\bin"
set ENTRYPOINT=mexFunction
set OUTDIR=.\
set LIB_NAME=ExtractDistanceTrajectory_mex
set MEX_NAME=ExtractDistanceTrajectory_mex
set MEX_EXT=.mexw64
call setEnv.bat
echo # Make settings for ExtractDistanceTrajectory > ExtractDistanceTrajectory_mex.mki
echo COMPILER=%COMPILER%>> ExtractDistanceTrajectory_mex.mki
echo COMPFLAGS=%COMPFLAGS%>> ExtractDistanceTrajectory_mex.mki
echo OPTIMFLAGS=%OPTIMFLAGS%>> ExtractDistanceTrajectory_mex.mki
echo DEBUGFLAGS=%DEBUGFLAGS%>> ExtractDistanceTrajectory_mex.mki
echo LINKER=%LINKER%>> ExtractDistanceTrajectory_mex.mki
echo LINKFLAGS=%LINKFLAGS%>> ExtractDistanceTrajectory_mex.mki
echo LINKOPTIMFLAGS=%LINKOPTIMFLAGS%>> ExtractDistanceTrajectory_mex.mki
echo LINKDEBUGFLAGS=%LINKDEBUGFLAGS%>> ExtractDistanceTrajectory_mex.mki
echo MATLAB_ARCH=%MATLAB_ARCH%>> ExtractDistanceTrajectory_mex.mki
echo OMPFLAGS= >> ExtractDistanceTrajectory_mex.mki
echo OMPLINKFLAGS= >> ExtractDistanceTrajectory_mex.mki
echo EMC_COMPILER=msvc150>> ExtractDistanceTrajectory_mex.mki
echo EMC_CONFIG=optim>> ExtractDistanceTrajectory_mex.mki
"C:\Program Files\MATLAB\R2018a\bin\win64\gmake" -j 1 -B -f ExtractDistanceTrajectory_mex.mk
