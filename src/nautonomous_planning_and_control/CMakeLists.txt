cmake_minimum_required(VERSION 2.8.3)
project(nautonomous_planning_and_control)

# CMake module(s) path
SET( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} )


find_package(catkin REQUIRED
roscpp
nautonomous_mpc_msgs
nautonomous_state_msgs
)

find_package( ACADO REQUIRED )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)



include_directories(
	include
	${PROJECT_SOURCE_DIR}/include/acado
	${PROJECT_SOURCE_DIR}/include/qpoases/INCLUDE
	${PROJECT_SOURCE_DIR}/include/qpoases/SRC
	. ${ACADO_INCLUDE_DIRS} 
	${catkin_INCLUDE_DIRS}
)

# acado libraries
add_library(acado_auxiliary_functions	include/acado/acado_auxiliary_functions.c)
add_library(acado_integrator		include/acado/acado_integrator.c)
add_library(acado_solver		include/acado/acado_solver.c)
add_library(acado_qpoases_interface	include/acado/acado_qpoases_interface.cpp)

# qpoases libraries
add_library(QProblemB			include/qpoases/SRC/QProblemB.cpp 	include/qpoases/SRC/QProblemB.ipp)
add_library(Bounds			include/qpoases/SRC/Bounds.cpp		include/qpoases/SRC/Bounds.ipp)
add_library(SubjectTo			include/qpoases/SRC/SubjectTo.cpp	include/qpoases/SRC/SubjectTo.ipp)
add_library(Indexlist			include/qpoases/SRC/Indexlist.cpp	include/qpoases/SRC/Indexlist.ipp)
add_library(Utils			include/qpoases/SRC/Utils.cpp		include/qpoases/SRC/Utils.ipp)
add_library(MessageHandling		include/qpoases/SRC/MessageHandling.cpp	include/qpoases/SRC/MessageHandling.ipp)

ADD_DEFINITIONS( -std=c++11 )

# add executable
add_executable(mpc src/mpc.cpp)
add_executable(mission_control src/mission_control.cpp)
#add_executable(mission_control2 src/mission_controlV2.cpp)
add_executable(ODEsolver src/ODEsolver.cpp)
#add_executable(ODEsolver2 src/ODEsolverV2.cpp)

# target link libraries
target_link_libraries(mpc ${catkin_LIBRARIES} acado_auxiliary_functions acado_integrator acado_solver acado_qpoases_interface)
target_link_libraries(mission_control ${catkin_LIBRARIES})
#target_link_libraries(mission_control2 ${catkin_LIBRARIES})
target_link_libraries(ODEsolver ${ACADO_SHARED_LIBRARIES} ${catkin_LIBRARIES})
SET_TARGET_PROPERTIES(ODEsolver PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} )
#target_link_libraries(ODEsolver2 ${catkin_LIBRARIES} )

target_link_libraries(acado_auxiliary_functions ${catkin_LIBRARIES} )
target_link_libraries(acado_integrator 		${catkin_LIBRARIES} )
target_link_libraries(acado_solver 		${catkin_LIBRARIES} acado_integrator)
target_link_libraries(acado_qpoases_interface 	${catkin_LIBRARIES} QProblemB)
target_link_libraries(QProblemB 		${catkin_LIBRARIES} Bounds)
target_link_libraries(Bounds	 		${catkin_LIBRARIES} SubjectTo)
target_link_libraries(SubjectTo 		${catkin_LIBRARIES} Indexlist)
target_link_libraries(Indexlist 		${catkin_LIBRARIES} Utils)
target_link_libraries(Utils 			${catkin_LIBRARIES} MessageHandling)

