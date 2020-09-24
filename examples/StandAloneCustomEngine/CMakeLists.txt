
INCLUDE_DIRECTORIES(
	${BULLET_PHYSICS_SOURCE_DIR}/src
	${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs
)

ADD_EXECUTABLE(App_CustomEngine_noGUI
		stand_alone_gen.cpp
)
target_link_libraries(App_CustomEngine_noGUI jsoncpp BulletCollision LinearMath Bullet3Common BulletInverseDynamics BulletInverseDynamicsUtils BulletDynamics BulletGenDynamics)
FILE( MAKE_DIRECTORY "${PROJECT_BINARY_DIR}/data" )


# IF (BULLET2_USE_TBB_MULTITHREADING AND WIN32)
#     # add a post build command to copy some dlls to the executable directory
#     set(TBB_VC_VER "vc12")
#     set(TBB_VC_ARCH "ia32")
#     # assume 32-bit build in VC12 for now
#     # checks can be added here at a later time
#     ADD_CUSTOM_COMMAND(TARGET App_ExampleBrowser POST_BUILD
#     COMMAND ${CMAKE_COMMAND} -E copy_if_different
#         "${BULLET2_TBB_INCLUDE_DIR}/../bin/${TBB_VC_ARCH}/${TBB_VC_VER}/tbb.dll"
#         $<TARGET_FILE_DIR:App_ExampleBrowser>)
#     ADD_CUSTOM_COMMAND(TARGET App_ExampleBrowser POST_BUILD
#     COMMAND ${CMAKE_COMMAND} -E copy_if_different
#         "${BULLET2_TBB_INCLUDE_DIR}/../bin/${TBB_VC_ARCH}/${TBB_VC_VER}/tbbmalloc.dll"
#         $<TARGET_FILE_DIR:App_ExampleBrowser>)
# ENDIF (BULLET2_USE_TBB_MULTITHREADING AND WIN32)


# IF (INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)
# 			SET_TARGET_PROPERTIES(App_ExampleBrowser PROPERTIES  DEBUG_POSTFIX "_Debug")
# 			SET_TARGET_PROPERTIES(App_ExampleBrowser PROPERTIES  MINSIZEREL_POSTFIX "_MinsizeRel")
# 			SET_TARGET_PROPERTIES(App_ExampleBrowser PROPERTIES  RELWITHDEBINFO_POSTFIX "_RelWithDebugInfo")
# ENDIF(INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)

