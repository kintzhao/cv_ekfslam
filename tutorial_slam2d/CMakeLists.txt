cmake_minimum_required(VERSION 2.8)
project(tutorial_slam2d)

       include(CheckCXXCompilerFlag)
      CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
      CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
      if(COMPILER_SUPPORTS_CXX11)
          set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
      elseif(COMPILER_SUPPORTS_CXX0X)
          set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
      else()
              message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
      endif()

      if(COMMAND cmake_policy)
        cmake_policy(SET CMP0003 NEW)
      endif(COMMAND cmake_policy)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules/")
message(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH})
# g2o #######################
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIR})
message("G2O_INCLUDE_DIRS ${G2O_INCLUDE_DIR}")
link_directories(${G2O_LIBRARY_DIRS})
link_libraries(${G2O_LIBRARIES})
message("G2o-libraries ${G2O_LIBRARIES}")

         # CSparse #######################
  find_package(CSparse REQUIRED)
  include_directories(${CSPARSE_INCLUDE_DIR})
  message("CSPARSE_INCLUDE_DIRS"${CSPARSE_INCLUDE_DIR})
  link_directories(${CSparse_LIBRARY_DIRS})
  link_libraries(${CSPARSE_LIBRARY})
  message("CSparse-libraries ${CSPARSE_LIBRARY}")


INCLUDE_DIRECTORIES(${CSPARSE_INCLUDE_DIR})

file(GLOB_RECURSE HEADERS_AR
        ${PROJECT_SOURCE_DIR}/class/*.h
        ${PROJECT_SOURCE_DIR}/class/*.cpp
)
message("HEADERS_AR = ${HEADERS_AR}")
ADD_LIBRARY(tutorial_slam2d_library   SHARED  ${HEADERS_AR})

SET_TARGET_PROPERTIES(tutorial_slam2d_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}tutorial_slam2d)

TARGET_LINK_LIBRARIES(tutorial_slam2d_library  ${G2O_LIBRARIES} )

ADD_EXECUTABLE(tutorial_slam2d
  tutorial_slam2d.cpp
)

SET_TARGET_PROPERTIES(tutorial_slam2d PROPERTIES OUTPUT_NAME tutorial_slam2d${EXE_POSTFIX})

TARGET_LINK_LIBRARIES(tutorial_slam2d tutorial_slam2d_library)
