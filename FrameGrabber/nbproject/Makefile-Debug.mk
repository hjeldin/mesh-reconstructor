#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Environment
MKDIR=mkdir
CP=cp
GREP=grep
NM=nm
CCADMIN=CCadmin
RANLIB=ranlib
CC=gcc
CCC=g++
CXX=g++
FC=gfortran
AS=as

# Macros
CND_PLATFORM=GNU-Linux-x86
CND_DLIB_EXT=so
CND_CONF=Debug
CND_DISTDIR=dist
CND_BUILDDIR=build

# Include project Makefile
include Makefile

# Object Directory
OBJECTDIR=${CND_BUILDDIR}/${CND_CONF}/${CND_PLATFORM}

# Object Files
OBJECTFILES= \
	${OBJECTDIR}/main.o \
	${OBJECTDIR}/ImageHelper.o


# C Compiler Flags
CFLAGS=

# CC Compiler Flags
CCFLAGS=
CXXFLAGS=

# Fortran Compiler Flags
FFLAGS=

# Assembler Flags
ASFLAGS=

# Link Libraries and Options
LDLIBSOPTIONS=-L../../../rgbdemo/build/lib -lopencv_contrib -lopencv_core -lopencv_features2d -lopencv_flann -lopencv_highgui -lopencv_imgproc -lOpenNI -lpcl_apps -lpcl_common -lpcl_features -lpcl_filters -lpcl_io -lpcl_io_ply -lpcl_kdtree -lpcl_keypoints -lpcl_octree -lpcl_range_image_border_extractor -lpcl_registration -lpcl_sample_consensus -lpcl_search -lpcl_segmentation -lpcl_surface -lpcl_tracking -lpcl_visualization -lboost_date_time-mt -lboost_date_time-mt -lboost_date_time -lboost_date_time -lboost_filesystem-mt -lboost_filesystem-mt -lboost_filesystem -lboost_filesystem -lboost_graph-mt -lboost_graph-mt -lboost_graph -lboost_graph -lboost_graph_parallel-mt -lboost_graph_parallel-mt -lboost_graph_parallel -lboost_graph_parallel -lboost_iostreams-mt -lboost_iostreams-mt -lboost_iostreams -lboost_iostreams -lboost_math_c99-mt -lboost_math_c99-mt -lboost_math_c99 -lboost_math_c99 -lboost_math_c99f-mt -lboost_math_c99f-mt -lboost_math_c99f -lboost_math_c99f -lboost_math_c99l-mt -lboost_math_c99l-mt -lboost_math_c99l -lboost_math_c99l -lboost_math_tr1-mt -lboost_math_tr1-mt -lboost_math_tr1 -lboost_math_tr1 -lboost_math_tr1f-mt -lboost_math_tr1f-mt -lboost_math_tr1f -lboost_math_tr1f -lboost_math_tr1l-mt -lboost_math_tr1l-mt -lboost_math_tr1l -lboost_math_tr1l -lboost_mpi-mt -lboost_mpi-mt -lboost_mpi -lboost_mpi -lboost_prg_exec_monitor-mt -lboost_prg_exec_monitor-mt -lboost_prg_exec_monitor -lboost_prg_exec_monitor -lboost_program_options-mt -lboost_program_options-mt -lboost_program_options -lboost_program_options -lboost_python-mt-py27 -lboost_python-mt-py27 -lboost_python-mt-py32 -lboost_python-mt-py32 -lboost_python-py27 -lboost_python-py27 -lboost_python-py32 -lboost_python-py32 -lboost_python -lboost_python -lboost_regex-mt -lboost_regex-mt -lboost_regex -lboost_regex -lboost_serialization-mt -lboost_serialization-mt -lboost_serialization -lboost_serialization -lboost_signals-mt -lboost_signals-mt -lboost_signals -lboost_signals -lboost_system-mt -lboost_system-mt -lboost_system -lboost_system -lboost_test_exec_monitor-mt -lboost_test_exec_monitor -lboost_thread-mt -lboost_thread-mt -lboost_thread -lboost_thread -lboost_unit_test_framework-mt -lboost_unit_test_framework-mt -lboost_unit_test_framework -lboost_unit_test_framework -lboost_wave-mt -lboost_wave-mt -lboost_wave -lboost_wave -lboost_wserialization-mt -lboost_wserialization-mt -lboost_wserialization -lboost_wserialization -lnestk

# Build Targets
.build-conf: ${BUILD_SUBPROJECTS}
	"${MAKE}"  -f nbproject/Makefile-${CND_CONF}.mk ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/framegrabber

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/framegrabber: ${OBJECTFILES}
	${MKDIR} -p ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}
	${LINK.cc} -o ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/framegrabber ${OBJECTFILES} ${LDLIBSOPTIONS} 

${OBJECTDIR}/main.o: main.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -g -I/usr/include/qt4/ -I/usr/include/qt4/QtCore -I/usr/include/qt4/Qt -I/usr/include/qt4/QtOpenGL -I/usr/include/qt4/QtSvg -I/usr/include/qt4/QtNetwork -I/usr/include/qt4/QtGui -I/usr/include/eigen3 -I../../../OpenNi/OpenNI/Include -I/usr/local/include/pcl-1.5 -I/usr/local/include/opencv2 -I/usr/local/include/opencv -I../../../rgbdemo/nestk/ -MMD -MP -MF $@.d -o ${OBJECTDIR}/main.o main.cpp

${OBJECTDIR}/ImageHelper.o: ImageHelper.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -g -I/usr/include/qt4/ -I/usr/include/qt4/QtCore -I/usr/include/qt4/Qt -I/usr/include/qt4/QtOpenGL -I/usr/include/qt4/QtSvg -I/usr/include/qt4/QtNetwork -I/usr/include/qt4/QtGui -I/usr/include/eigen3 -I../../../OpenNi/OpenNI/Include -I/usr/local/include/pcl-1.5 -I/usr/local/include/opencv2 -I/usr/local/include/opencv -I../../../rgbdemo/nestk/ -MMD -MP -MF $@.d -o ${OBJECTDIR}/ImageHelper.o ImageHelper.cpp

# Subprojects
.build-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r ${CND_BUILDDIR}/${CND_CONF}
	${RM} ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/framegrabber

# Subprojects
.clean-subprojects:

# Enable dependency checking
.dep.inc: .depcheck-impl

include .dep.inc
