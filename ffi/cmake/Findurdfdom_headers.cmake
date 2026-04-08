include(FindPackageHandleStandardArgs)

find_path(
  urdfdom_headers_INCLUDE_DIR
  NAMES urdf_model/model.h
  HINTS "${PINOCCHIO_DEP_PREFIX}/include/urdfdom_headers" "${PINOCCHIO_DEP_PREFIX}/include")

find_path(
  urdfdom_parser_INCLUDE_DIR
  NAMES urdf_parser/urdf_parser.h
  HINTS "${PINOCCHIO_DEP_PREFIX}/include/urdfdom" "${PINOCCHIO_DEP_PREFIX}/include")

set(
  urdfdom_headers_INCLUDE_DIRS
  "${urdfdom_headers_INCLUDE_DIR};${urdfdom_parser_INCLUDE_DIR}")
set(urdfdom_headers_VERSION "4.0.1")

find_package_handle_standard_args(
  urdfdom_headers
  REQUIRED_VARS urdfdom_headers_INCLUDE_DIR urdfdom_parser_INCLUDE_DIR
  VERSION_VAR urdfdom_headers_VERSION)

if(urdfdom_headers_FOUND AND NOT TARGET urdfdom_headers::urdfdom_headers)
  add_library(urdfdom_headers::urdfdom_headers INTERFACE IMPORTED)
  set_target_properties(
    urdfdom_headers::urdfdom_headers
    PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${urdfdom_headers_INCLUDE_DIRS}")
endif()
