include(FindPackageHandleStandardArgs)

find_package(urdfdom_headers REQUIRED)

find_library(
  urdfdom_sensor_LIBRARY
  NAMES urdfdom_sensor
  HINTS "${PINOCCHIO_DEP_PREFIX}/lib")
find_library(
  urdfdom_model_state_LIBRARY
  NAMES urdfdom_model_state
  HINTS "${PINOCCHIO_DEP_PREFIX}/lib")
find_library(
  urdfdom_model_LIBRARY
  NAMES urdfdom_model
  HINTS "${PINOCCHIO_DEP_PREFIX}/lib")
find_library(
  urdfdom_world_LIBRARY
  NAMES urdfdom_world
  HINTS "${PINOCCHIO_DEP_PREFIX}/lib")

set(
  urdfdom_LIBRARIES
  ${urdfdom_sensor_LIBRARY}
  ${urdfdom_model_state_LIBRARY}
  ${urdfdom_model_LIBRARY}
  ${urdfdom_world_LIBRARY})
set(
  urdfdom_INCLUDE_DIRS
  "${PINOCCHIO_DEP_PREFIX}/include/urdfdom_headers;${PINOCCHIO_DEP_PREFIX}/include/urdfdom")
set(urdfdom_VERSION "${urdfdom_headers_VERSION}")

find_package_handle_standard_args(
  urdfdom
  REQUIRED_VARS urdfdom_LIBRARIES urdfdom_INCLUDE_DIRS
  VERSION_VAR urdfdom_VERSION)

if(urdfdom_FOUND AND NOT TARGET urdfdom::urdf_parser)
  add_library(urdfdom::urdf_parser INTERFACE IMPORTED)
  set_target_properties(
    urdfdom::urdf_parser
    PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${urdfdom_INCLUDE_DIRS}"
               INTERFACE_LINK_LIBRARIES "${urdfdom_LIBRARIES}")
endif()
