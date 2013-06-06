# - Config file for the Armadillo package
# It defines the following variables
#  ARMADILLO_INCLUDE_DIRS - include directories for Armadillo
#  ARMADILLO_LIBRARY_DIRS - library directories for Armadillo (normally not used!)
#  ARMADILLO_LIBRARIES    - libraries to link against

# Tell the user project where to find our headers and libraries
set(ARMADILLO_INCLUDE_DIRS "/users/guest/m/mikeac89/cs470/bzrflag/armadillo;/users/guest/m/mikeac89/cs470/bzrflag/armadillo")
set(ARMADILLO_LIBRARY_DIRS "/users/guest/m/mikeac89/cs470/bzrflag/armadillo")

# Our library dependencies (contains definitions for IMPORTED targets)
include("/users/guest/m/mikeac89/cs470/bzrflag/armadillo/ArmadilloLibraryDepends.cmake")

# These are IMPORTED targets created by ArmadilloLibraryDepends.cmake
set(ARMADILLO_LIBRARIES armadillo)

