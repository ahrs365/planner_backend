##
##	This file is part of qp42.
##
##	qp42 -- An Implementation of the Online Active Set Strategy.
##	Copyright (C) 2012 by Janick Frasch, Hans Joachim Ferreau et al. 
##	All rights reserved.
##
##	qp42 is free software; you can redistribute it and/or
##	modify it under the terms of the GNU Lesser General Public
##	License as published by the Free Software Foundation; either
##	version 2.1 of the License, or (at your option) any later version.
##
##	qp42 is distributed in the hope that it will be useful,
##	but WITHOUT ANY WARRANTY; without even the implied warranty of
##	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
##	See the GNU Lesser General Public License for more details.
##
##	You should have received a copy of the GNU Lesser General Public
##	License along with qp42; if not, write to the Free Software
##	Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
##



##
##	Filename:  src/make_linux
##	Author:    Janick Frasch, Hans Joachim Ferreau
##	Version:   1.0beta
##	Date:      2012
##


##
##	definitions for compiling with gcc under linux
##

CC = gcc
CPP = g++
AR  = ar
RM  = rm

OBJEXT = o
LIBEXT = a
EXE =
DEF_TARGET = -o $@

OMPFLAGS = ##-fopenmp		# avoid automatic gcc parallelization
DEBUGFLAGS = -g -ggdb -D__DEBUG__ -Wshadow -Wall -pedantic

CCFLAGS = ${DEBUGFLAGS} -finline-functions -DLINUX -D__DEBUG__ -std=c99 ${OMPFLAGS}		##C99 temporary to avoid warnings	
##CcFLAGS = -Wall -pedantic -Wshadow -O3 -finline-functions -DLINUX -D__SUPPRESS_ALL_OUTPUT__ -U__MEASURE_TIMINGS__ -U__ANALYZE_FACTORIZATION__
CPPFLAGS = ${DEBUGFLAGS} -finline-functions -DLINUX -D__DEBUG__ ${OMPFLAGS} 

QPDUNES_LIB         =  -L${SRCDIR} -lqpdunes

MPCDUNES_LIB        =  -L${INTERFACEDIR}/mpc -lmpcdunes

##QPOASES_LIB			=  -static -L${QPOASESDIR}/bin -lqpOASES ${QPOASESDIR}/src/BLASReplacement.o ${QPOASESDIR}/src/LAPACKReplacement.o
QPOASES_LIB			=  ${QPOASESDIR}/bin/libqpOASES.${LIBEXT} ${QPOASESDIR}/src/BLASReplacement.${OBJEXT} ${QPOASESDIR}/src/LAPACKReplacement.${OBJEXT}

LIBS         =  -lm


##
##	end of file
##
