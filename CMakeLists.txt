# SPDX-License-Identifier: MIT-0

cmake_minimum_required(VERSION 3.10)
project(inscribed_ellipsoid_test)

find_package(drake CONFIG REQUIRED PATHS /opt/drake)

add_executable(inscribed_ellipsoid_test inscribed_ellipsoid_test.cc)
target_link_libraries(inscribed_ellipsoid_test drake::drake)
set_property(TARGET inscribed_ellipsoid_test PROPERTY CXX_STANDARD 20)