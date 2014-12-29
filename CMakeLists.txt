# Copyright 2014, CNRS
#
# Author: Ganesh Kumar
#
# This file is part of vsa1.
# pam is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# vsa1 is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# pam. If not, see <http://www.gnu.org/licenses/>.

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/eigen.cmake)
#INCLUDE(cmake/findxenomai.cmake)

SET(PROJECT_NAME vsa1)
SET(PROJECT_DESCRIPTION
  "General dynamic simulation of VSA with 1 axis.")
SET(PROJECT_URL "")

SET(CUSTOM_HEADER_DIR "${PROJECT_NAME}")

SET(${PROJECT_NAME}_HEADERS
  include/integrate-dynamics.h
  include/port.h
  include/plantmodel.h
  include/serverudp3.h
  include/system-dynamics.h
)
SET(CXX_DISABLE_WERROR TRUE)

ADD_REQUIRED_DEPENDENCY ("libxenomai_native")
ADD_REQUIRED_DEPENDENCY ("libxenomai_rtdm")

SETUP_PROJECT()

#SEARCH_FOR_BOOST()
SEARCH_FOR_EIGEN()

ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(application)
SETUP_PROJECT_FINALIZE()