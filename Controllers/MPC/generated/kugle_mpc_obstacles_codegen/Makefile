#
#    This file was auto-generated using the ACADO Toolkit.
#    
#    While ACADO Toolkit is free software released under the terms of
#    the GNU Lesser General Public License (LGPL), the generated code
#    as such remains the property of the user who used ACADO Toolkit
#    to generate this code. In particular, user dependent data of the code
#    do not inherit the GNU LGPL license. On the other hand, parts of the
#    generated code that are a direct copy of source code from the
#    ACADO Toolkit or the software tools it is based on, remain, as derived
#    work, automatically covered by the LGPL license.
#    
#    ACADO Toolkit is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
#    


UNAME := $(shell uname)

LDLIBS = -lm -lstdc++
ifeq ($(UNAME), Linux)
	LDLIBS += -lrt
endif

CCACHE_APP := $(shell which ccache 2>/dev/null)

CFLAGS = -O3 -finline-functions -I. -I./qpoases3 -I./qpoases3/include -I./qpoases3/src -D__CODE_GENERATION__
CXXFLAGS = -O3 -finline-functions -I. -I./qpoases3 -I./qpoases3/include -I./qpoases3/src
CC     = $(CCACHE_APP) gcc
CXX    = $(CCACHE_APP) g++

OBJECTS = \
	./qpoases3/src/Bounds.o \
	./qpoases3/src/Constraints.o \
	./qpoases3/src/Indexlist.o \
	./qpoases3/src/Matrices.o \
	./qpoases3/src/MessageHandling.o \
	./qpoases3/src/Options.o \
	./qpoases3/src/Flipper.o \
	./qpoases3/src/QProblem.o \
	./qpoases3/src/QProblemB.o \
	./qpoases3/src/Utils.o \
	acado_qpoases3_interface.o \
	acado_integrator.o \
	acado_solver.o \
	acado_auxiliary_functions.o

.PHONY: all
all: libacado_exported_rti.a test

test: ${OBJECTS} test.o

acado_qpoases3_interface.o  : acado_qpoases3_interface.h
acado_solver.o              : acado_common.h
acado_integrator.o          : acado_common.h
acado_auxiliary_functions.o : acado_common.h \
                              acado_auxiliary_functions.h
test.o                      : acado_common.h \
                              acado_qpoases3_interface.h \
                              acado_auxiliary_functions.h

libacado_exported_rti.a: ${OBJECTS}
	ar r $@ $?

${OBJECTS} : acado_qpoases3_interface.h

.PHONY : clean
clean :
	-rm -f *.o *.a ./qpoases3/*.o ./qpoases3/src/*.o ./qpoases3/src/*.a test
