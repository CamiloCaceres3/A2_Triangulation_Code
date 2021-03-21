# Install script for directory: C:/Users/dj_do/OneDrive/GEOMATICS/GEO1016 Photogrammetry/Assignments/A2_collab_working

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "C:/Users/dj_do/OneDrive/GEOMATICS/GEO1016 Photogrammetry/Assignments/A2_collab_working/out/install/x64-Debug (default)")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("C:/Users/dj_do/OneDrive/GEOMATICS/GEO1016 Photogrammetry/Assignments/A2_collab_working/out/build/x64-Debug (default)/3rd_party/cmake_install.cmake")
  include("C:/Users/dj_do/OneDrive/GEOMATICS/GEO1016 Photogrammetry/Assignments/A2_collab_working/out/build/x64-Debug (default)/easy3d/cmake_install.cmake")
  include("C:/Users/dj_do/OneDrive/GEOMATICS/GEO1016 Photogrammetry/Assignments/A2_collab_working/out/build/x64-Debug (default)/Triangulation/cmake_install.cmake")
  include("C:/Users/dj_do/OneDrive/GEOMATICS/GEO1016 Photogrammetry/Assignments/A2_collab_working/out/build/x64-Debug (default)/Tutorial_NonlinearLeastSquares/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "C:/Users/dj_do/OneDrive/GEOMATICS/GEO1016 Photogrammetry/Assignments/A2_collab_working/out/build/x64-Debug (default)/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
