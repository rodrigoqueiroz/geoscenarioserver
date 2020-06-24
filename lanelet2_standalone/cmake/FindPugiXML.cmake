find_library(PugiXML_LIBRARIES
  NAMES pugixml)
find_path(PugiXML_INCLUDE_DIRS
  NAMES pugixml.hpp
  PATH_SUFFIXES pugixml)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(pugixml DEFAULT_MSG
  PugiXML_LIBRARIES
  PugiXML_INCLUDE_DIRS)
message(STATUS "PugiXML include DIRS: " ${PugiXML_INCLUDE_DIRS} ", PugiXML library: " ${PugiXML_LIBRARIES})
