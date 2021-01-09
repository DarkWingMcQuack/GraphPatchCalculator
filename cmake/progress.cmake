include(ExternalProject)
include(GNUInstallDirs)

set(CMAKE_ARGS
  -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
  -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
  -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER})

ExternalProject_Add(progress-cpp-project
  PREFIX deps/progress-cpp
  GIT_REPOSITORY https://github.com/DarkWingMcQuack/progress-cpp
  DOWNLOAD_DIR ${CMAKE_BINARY_DIR}/downloads
  PATCH_COMMAND cmake -E make_directory <SOURCE_DIR>/win32-deps/include
  UPDATE_COMMAND ""
  CMAKE_ARGS ${CMAKE_ARGS}
  # Overwtire build and install commands to force Release build on MSVC.
  BUILD_COMMAND cmake --build <BINARY_DIR> --config Release
  INSTALL_COMMAND cmake --build <BINARY_DIR> --config Release --target install
  )

ExternalProject_Get_Property(progress-cpp-project INSTALL_DIR)
set(PROGRESS_CPP_INCLUDE_DIR ${INSTALL_DIR}/include/progresscpp)

unset(INSTALL_DIR)
unset(CMAKE_ARGS)
