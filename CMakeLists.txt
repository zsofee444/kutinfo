cmake_minimum_required(VERSION 3.8) # CXX_STANDARD 17
SET(LATEX_OUTPUT_PATH documentation)

project(doublependulum LANGUAGES CXX)

if (MSVC)
  string(REGEX REPLACE "/W[0-9]" "" CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS})
endif (MSVC)

add_executable(${PROJECT_NAME} src/Main.cpp)

set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 17
                                                 CXX_STANDARD_REQUIRED ON
                                                 CXX_EXTENSIONS OFF)

target_compile_options(${PROJECT_NAME} PRIVATE $<$<OR:$<CXX_COMPILER_ID:GNU>,$<CXX_COMPILER_ID:Clang>>:-Wall -Wextra -pedantic>
                                               $<$<CXX_COMPILER_ID:MSVC>:/W4 /permissive->)

# documentation
include(cmake/UseLATEX.cmake)

add_latex_document(doc/doublependulum.tex
                   CONFIGURE doc/doublependulum.tex
                   TARGET_NAME doc
                   IMAGE_DIRS img
DEPENDS doc/doublependulum.tex
