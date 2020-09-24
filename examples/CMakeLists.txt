SUBDIRS( HelloWorld BasicDemo )
IF(BUILD_BULLET3)
	SUBDIRS( ExampleBrowser RobotSimulator SharedMemory ThirdPartyLibs/Gwen ThirdPartyLibs/BussIK ThirdPartyLibs/clsocket OpenGLWindow TwoJoint StandAloneCustomEngine)
ENDIF()

IF(BUILD_PYBULLET)
	SUBDIRS(pybullet)
ENDIF(BUILD_PYBULLET)
