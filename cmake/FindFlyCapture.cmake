if(MSVC)
  if(CMAKE_CL_64)

    if(EXISTS "C:\\Program Files\\Point Grey Research\\FlyCapture2")
      set(FLYCAPTURE_ROOT "C:\\Program Files\\Point Grey Research\\FlyCapture2")

    else(EXISTS "C:\\Program Files\\Point Grey Research\\FlyCapture2")

      if(EXISTS "C:\\Program Files (x86)\\Point Grey Research\\FlyCapture2")
        set(FLYCAPTURE_ROOT "C:\\Program Files (x86)\\Point Grey Research\\FlyCapture2")
      endif(EXISTS "C:\\Program Files (x86)\\Point Grey Research\\FlyCapture2")
    endif(EXISTS "C:\\Program Files\\Point Grey Research\\FlyCapture2")

  else(CMAKE_CL_64)

    if(EXISTS "C:\\Program Files (x86)\\Point Grey Research\\FlyCapture2")
      set(FLYCAPTURE_ROOT "C:\\Program Files (x86)\\Point Grey Research\\FlyCapture2")

    else(EXISTS "C:\\Program Files (x86)\\Point Grey Research\\FlyCapture2")

      if(EXISTS "C:\\Program Files\\Point Grey Research\\FlyCapture2")
        set(FLYCAPTURE_ROOT "C:\\Program Files\\Point Grey Research\\FlyCapture2")
      endif(EXISTS "C:\\Program Files\\Point Grey Research\\FlyCapture2")
    endif(EXISTS "C:\\Program Files (x86)\\Point Grey Research\\FlyCapture2")

  endif(CMAKE_CL_64)
  message(STATUS "FLYCAPTURE_ROOT : ${FLYCAPTURE_ROOT}")

  if(FLYCAPTURE_ROOT)
    set(FLYCAPTURE_INCLUDE_DIR "${FLYCAPTURE_ROOT}/include")
    message(STATUS "FLYCAPTURE_INCLUDE_DIR : ${FLYCAPTURE_INCLUDE_DIR}")
  endif(FLYCAPTURE_ROOT)

  if(FLYCAPTURE_INCLUDE_DIR)
    if(CMAKE_CL_64)
      if(EXISTS "${FLYCAPTURE_ROOT}/lib64")
        set(FLYCAPTURE_LIBRARY_DIR "${FLYCAPTURE_ROOT}/lib64/vs2015")
        set(FLYCAPTURE_DLL_DIR "${FLYCAPTURE_ROOT}/bin64/vs2015")

      else(EXISTS "${FLYCAPTURE_ROOT}/lib64")
        set(FLYCAPTURE_LIBRARY_DIR "${FLYCAPTURE_ROOT}/lib")
        set(FLYCAPTURE_DLL_DIR "${FLYCAPTURE_ROOT}/bin")
      endif(EXISTS "${FLYCAPTURE_ROOT}/lib64")

    else(CMAKE_CL_64)
      if(EXISTS "${FLYCAPTURE_ROOT}/lib")
        set(FLYCAPTURE_LIBRARY_DIR "${FLYCAPTURE_ROOT}/lib")
        set(FLYCAPTURE_DLL_DIR "${FLYCAPTURE_ROOT}/bin")

      else(EXISTS "${FLYCAPTURE_ROOT}/lib")
        set(FLYCAPTURE_LIBRARY_DIR "${FLYCAPTURE_ROOT}/lib64")
        set(FLYCAPTURE_DLL_DIR "${FLYCAPTURE_ROOT}/bin64")
      endif(EXISTS "${FLYCAPTURE_ROOT}/lib")

    endif(CMAKE_CL_64)

    find_library(FLYCAPTURE2_LIB
      NAMES FlyCapture2_v140
      PATHS
	    "${FLYCAPTURE_LIBRARY_DIR}"
    )
    find_library(FLYCAPTURE2_DLL
      NAMES FlyCapture2_v140
      PATHS
	    "${FLYCAPTURE_DLL_DIR}"
    )
  endif(FLYCAPTURE_INCLUDE_DIR)

else(MSVC)

  find_path(FLYCAPTURE_INCLUDE_DIR
    NAMES FlyCapture2.h
    PATHS "/usr/include/flycapture"
  )

  if(FLYCAPTURE_INCLUDE_DIR)
    set(FLYCAPTURE_LIBRARY_DIR /usr/lib/)
    find_library(FLYCAPTURE2_LIB
	    NAMES flycapture
	    PATHS
	    "${FLYCAPTURE_LIBRARY_DIR}"
    )
  endif(FLYCAPTURE_INCLUDE_DIR)
endif(MSVC)

message(STATUS "FLYCAPTURE_LIBRARY_DIR : ${FLYCAPTURE_LIBRARY_DIR}")
message(STATUS "FLYCAPTURE2_LIB : ${FLYCAPTURE2_LIB}")

if(FLYCAPTURE2_LIB)
  set(FLYCAPTURE_FOUND TRUE)
else(FLYCAPTURE2_LIB)
  message(STATUS "Warning: cannot find FlyCapture2")
endif(FLYCAPTURE2_LIB)