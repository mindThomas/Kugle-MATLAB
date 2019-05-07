START_DIR = C:\Users\Thomas\DOCUME~1\KUGLE-~1\CONTRO~1\MPC\GENERA~1

MATLAB_ROOT = C:\PROGRA~1\MATLAB\R2018a
MAKEFILE = ExtractDistanceTrajectory_mex.mk

include ExtractDistanceTrajectory_mex.mki


SRC_FILES =  \
	ExtractDistanceTrajectory_data.c \
	ExtractDistanceTrajectory_initialize.c \
	ExtractDistanceTrajectory_terminate.c \
	ExtractDistanceTrajectory.c \
	eml_int_forloop_overflow_check.c \
	repmat.c \
	RotateTrajectory.c \
	power.c \
	scalexpAlloc.c \
	error.c \
	mod.c \
	sort1.c \
	sortIdx.c \
	_coder_ExtractDistanceTrajectory_info.c \
	_coder_ExtractDistanceTrajectory_api.c \
	_coder_ExtractDistanceTrajectory_mex.c \
	ExtractDistanceTrajectory_emxutil.c \
	c_mexapi_version.c

MEX_FILE_NAME_WO_EXT = ExtractDistanceTrajectory_mex
MEX_FILE_NAME = $(MEX_FILE_NAME_WO_EXT).mexw64
TARGET = $(MEX_FILE_NAME)

SYS_LIBS = libmwblas.lib 


#
#====================================================================
# gmake makefile fragment for building MEX functions using MSVC
# Copyright 2007-2016 The MathWorks, Inc.
#====================================================================
#
SHELL = cmd
OBJEXT = obj
CC = $(COMPILER)
LD = $(LINKER)
.SUFFIXES: .$(OBJEXT)

OBJLISTC = $(SRC_FILES:.c=.$(OBJEXT))
OBJLIST  = $(OBJLISTC:.cpp=.$(OBJEXT))

TARGETMT = $(TARGET).manifest
MEX = $(TARGETMT)
STRICTFP = /fp:strict

target: $(MEX)

MATLAB_INCLUDES = /I "$(MATLAB_ROOT)\simulink\include"
MATLAB_INCLUDES+= /I "$(MATLAB_ROOT)\toolbox\shared\simtargets"
SYS_INCLUDE = $(MATLAB_INCLUDES)

# Additional includes

SYS_INCLUDE += /I "$(START_DIR)\codegen\mex\ExtractDistanceTrajectory"
SYS_INCLUDE += /I "$(START_DIR)"
SYS_INCLUDE += /I ".\interface"
SYS_INCLUDE += /I "$(MATLAB_ROOT)\extern\include"
SYS_INCLUDE += /I "."

DIRECTIVES = $(MEX_FILE_NAME_WO_EXT)_mex.arf
COMP_FLAGS = $(COMPFLAGS) $(OMPFLAGS)
LINK_FLAGS = $(filter-out /export:mexFunction, $(LINKFLAGS))
LINK_FLAGS += /NODEFAULTLIB:LIBCMT
ifeq ($(EMC_CONFIG),optim)
  COMP_FLAGS += $(OPTIMFLAGS) $(STRICTFP)
  LINK_FLAGS += $(LINKOPTIMFLAGS)
else
  COMP_FLAGS += $(DEBUGFLAGS)
  LINK_FLAGS += $(LINKDEBUGFLAGS)
endif
LINK_FLAGS += $(OMPLINKFLAGS)
LINK_FLAGS += /OUT:$(TARGET)
LINK_FLAGS +=  /LIBPATH:"$(MATLAB_ROOT)\extern\lib\win64\microsoft"

CFLAGS = $(COMP_FLAGS)   $(USER_INCLUDE) $(SYS_INCLUDE)
CPPFLAGS = $(COMP_FLAGS)   $(USER_INCLUDE) $(SYS_INCLUDE)

%.$(OBJEXT) : %.c
	$(CC) $(CFLAGS) "$<"

%.$(OBJEXT) : %.cpp
	$(CC) $(CPPFLAGS) "$<"

# Additional sources

%.$(OBJEXT) : $(START_DIR)/%.c
	$(CC) $(CFLAGS) "$<"

%.$(OBJEXT) : $(START_DIR)\codegen\mex\ExtractDistanceTrajectory/%.c
	$(CC) $(CFLAGS) "$<"

%.$(OBJEXT) : interface/%.c
	$(CC) $(CFLAGS) "$<"



%.$(OBJEXT) : $(START_DIR)/%.cpp
	$(CC) $(CPPFLAGS) "$<"

%.$(OBJEXT) : $(START_DIR)\codegen\mex\ExtractDistanceTrajectory/%.cpp
	$(CC) $(CPPFLAGS) "$<"

%.$(OBJEXT) : interface/%.cpp
	$(CC) $(CPPFLAGS) "$<"



$(TARGET): $(OBJLIST) $(MAKEFILE) $(DIRECTIVES)
	$(LD) $(LINK_FLAGS) $(OBJLIST) $(USER_LIBS) $(SYS_LIBS) @$(DIRECTIVES)
	@cmd /C "echo Build completed using compiler $(EMC_COMPILER)"

$(TARGETMT): $(TARGET)
	mt -outputresource:"$(TARGET);2" -manifest "$(TARGET).manifest"

#====================================================================

