
INCLUDE_DIRECTORIES(
	${BULLET_PHYSICS_SOURCE_DIR}/src
)

SET(
	BulletGenModel_SRCS
	./btGenModel/BaseObject.cpp
	./btGenModel/ColObjBase.cpp
	./btGenModel/EulerAngelRotationMatrix.cpp
	./btGenModel/Joint.cpp
	./btGenModel/Link.cpp
	./btGenModel/Printer.cpp
	./btGenModel/RobotCollider.cpp
	./btGenModel/RobotModel.cpp
	./btGenModel/RobotModelDynamics.cpp
	./btGenModel/RootJoint.cpp
	./btGenModel/LimitRootJoint.cpp
	./btGenModel/SimObj.cpp
	./btGenModel/tools.cpp
	btGenModel/Tinyxml/tinystr.cpp
	btGenModel/Tinyxml/tinyxml.cpp
	btGenModel/Tinyxml/tinyxmlerror.cpp
	btGenModel/Tinyxml/tinyxmlparser.cpp
)

SET(
	BulletGenSolver_SRCS
	./btGenSolver/ConstraintData.cpp
	./btGenSolver/ContactSolver.cpp
	./btGenSolver/ContactSolverTest.cpp
	./btGenSolver/LCPSolverBase.cpp
	./btGenSolver/SequentialImpulsePlugin.cpp
	./btGenSolver/NativeLemke/Lemke.cpp
	./btGenSolver/NativeLemke/NativeLemkeLCPSolver.cpp
	./btGenSolver/btPath/btPath.cpp
	./btGenSolver/odeDantzig/error.cpp
	./btGenSolver/odeDantzig/fastdot.cpp
	./btGenSolver/odeDantzig/fastldlt.cpp
	./btGenSolver/odeDantzig/fastlsolve.cpp
	./btGenSolver/odeDantzig/fastltsolve.cpp
	./btGenSolver/odeDantzig/lcp.cpp
	./btGenSolver/odeDantzig/Lemke.cpp
	./btGenSolver/odeDantzig/matrix.cpp
	./btGenSolver/odeDantzig/misc.cpp
	./btGenSolver/odeDantzig/odeDantzigLCPSolver.cpp
)

SET(
	BulletGenUtil_SRCS
	btGenUtil/JsonUtil.cpp
	btGenUtil/MathUtil.cpp
	btGenUtil/TimeUtil.cpp
)

SET(
	BulletGenPDControler_SRCS
	./btGenController/btGenContactAwareController.cpp
	./btGenController/btGenTargetCalculator.cpp
	./btGenController/ContactAdaptionFeature/btGenFeature.cpp
	./btGenController/ContactAdaptionFeature/btGenFeatureUtils.cpp
	./btGenController/PDController/btGenPDController.cpp
	./btGenController/FBFCalculator/btGenFBFConstraint.cpp
	./btGenController/FBFCalculator/btGenFBFEnergyTerm.cpp
	./btGenController/FBFCalculator/btGenFrameByFrameCalculator.cpp
	./btGenController/FBFCalculator/btGenFrameByFrameCalculatorTerm.cpp
	./btGenController/FBFCalculator/btCharContactPoint.cpp
	./btGenController/NQRCalculator/btGenNQRCalculator.cpp
	# ./btGenController/QPSolver/MatlabQPSolver.cpp
	./btGenController/QPSolver/QuadProgQPSolver.cpp
	./btGenController/QPSolver/eiquadprog.hpp
	./btGenController/btTraj.cpp
)

# if (UNIX)
#     find_package(Matlab REQUIRED)
#     if(Matlab_FOUND)
#         include_directories(${Matlab_INCLUDE_DIRS})
#     endif()

#     # add matlab QP solver
#     find_library(qp_lib
#                 NAMES libQUADASSINPROG.so libQUADASSINPROG.dylib
# 				PATHS "./btGenController/QPSolver/SQPMatlab/"
				
# 				# /home/xudong/Projects/bullet3/src/BulletGenDynamics
				

#     )

#     # mwmclmcrrt就是matlab的runtime
#     set(matlab_lib ${qp_lib} mwmclmcrrt)
# endif()

SET(BulletGenDynamics_SRCS
	btGenWorld.cpp
	${BulletGenPDControler_SRCS}
	${BulletGenSolver_SRCS}
	${BulletGenModel_SRCS}
	${BulletGenUtil_SRCS}
)

SET(BulletGenDynamics_HDRS
	btGenWorld.h
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)
link_libraries(${JSONCPP_LIBRARIES})

ADD_LIBRARY(BulletGenDynamics ${BulletGenDynamics_SRCS})
target_link_libraries(BulletGenDynamics ${JSONCPP_LIBRARIES})
target_link_libraries(BulletGenDynamics ${matlab_lib})

if (BUILD_SHARED_LIBS)
  target_link_libraries(BulletGenDynamics Bullet3Geometry)
endif ()
SET_TARGET_PROPERTIES(BulletGenDynamics PROPERTIES VERSION ${BULLET_VERSION})
SET_TARGET_PROPERTIES(BulletGenDynamics PROPERTIES SOVERSION ${BULLET_VERSION})

IF (INSTALL_LIBS)
	IF (NOT INTERNAL_CREATE_DISTRIBUTABLE_MSVC_PROJECTFILES)
		#FILES_MATCHING requires CMake 2.6
		IF (${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} GREATER 2.5)
			IF (APPLE AND BUILD_SHARED_LIBS AND FRAMEWORK)
				INSTALL(TARGETS BulletGenDynamics DESTINATION .)
			ELSE (APPLE AND BUILD_SHARED_LIBS AND FRAMEWORK)
				INSTALL(TARGETS BulletGenDynamics
					RUNTIME DESTINATION bin
					LIBRARY DESTINATION lib${LIB_SUFFIX}
					ARCHIVE DESTINATION lib${LIB_SUFFIX})
				INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
DESTINATION ${INCLUDE_INSTALL_DIR} FILES_MATCHING PATTERN "*.h"  PATTERN
".svn" EXCLUDE PATTERN "CMakeFiles" EXCLUDE)
			ENDIF (APPLE AND BUILD_SHARED_LIBS AND FRAMEWORK)
		ENDIF (${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} GREATER 2.5)

		IF (APPLE AND BUILD_SHARED_LIBS AND FRAMEWORK)
			SET_TARGET_PROPERTIES(BulletGenDynamics PROPERTIES FRAMEWORK true)
			SET_TARGET_PROPERTIES(BulletGenDynamics PROPERTIES PUBLIC_HEADER "${BulletGenDynamics_HDRS}")
			# Have to list out sub-directories manually:
			#todo
			#SET_PROPERTY(SOURCE ${BulletGenDynamicsBroadPhase_HDRS} PROPERTY MACOSX_PACKAGE_LOCATION Headers/BroadPhaseCollision)

		ENDIF (APPLE AND BUILD_SHARED_LIBS AND FRAMEWORK)
	ENDIF (NOT INTERNAL_CREATE_DISTRIBUTABLE_MSVC_PROJECTFILES)
ENDIF (INSTALL_LIBS)
