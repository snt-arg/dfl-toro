#!/bin/bash
set -e

export TENSORRT_VERSION=$(echo "${TENSORRT_VERSION}" | cut -d . -f 1-3)

cat <<EOF

=====================
== NVIDIA TensorRT ==
=====================

NVIDIA Release ${NVIDIA_TENSORRT_VERSION} (build ${NVIDIA_BUILD_ID})

NVIDIA TensorRT ${TENSORRT_VERSION} (c) 2016-2021, NVIDIA CORPORATION.  All rights reserved.
Container image (c) 2021, NVIDIA CORPORATION.  All rights reserved.

https://developer.nvidia.com/tensorrt

This container image and its contents are governed by the NVIDIA Deep Learning Container License.
By pulling and using the container, you accept the terms and conditions of this license:
https://developer.nvidia.com/ngc/nvidia-deep-learning-container-license

To install Python sample dependencies, run /opt/tensorrt/python/python_setup.sh

To install the open-source samples corresponding to this TensorRT release version run /opt/tensorrt/install_opensource.sh.
To build the open source parsers, plugins, and samples for current top-of-tree on master or a different branch, run /opt/tensorrt/install_opensource.sh -b <branch>
See https://github.com/NVIDIA/TensorRT for more information.
EOF

if [[ "$(find -L /usr -name libcuda.so.1 2>/dev/null | grep -v "compat") " == " " || "$(ls /dev/nvidiactl 2>/dev/null) " == " " ]]; then
  echo
  echo "WARNING: The NVIDIA Driver was not detected.  GPU functionality will not be available."
  echo "   Use 'nvidia-docker run' to start this container; see"
  echo "   https://github.com/NVIDIA/nvidia-docker/wiki/nvidia-docker ."
else
  ( MINSMVER=30 /usr/local/bin/checkSMVER.sh )
  DRIVER_VERSION=$(sed -n 's/^NVRM.*Kernel Module *\([0-9.]*\).*$/\1/p' /proc/driver/nvidia/version 2>/dev/null || true)
  if [[ ! "$DRIVER_VERSION" =~ ^[0-9]*.[0-9]*(.[0-9]*)?$ ]]; then
    echo "Failed to detect NVIDIA driver version."
  elif [[ "${DRIVER_VERSION%%.*}" -lt "${CUDA_DRIVER_VERSION%%.*}" ]]; then
    if [[ "${_CUDA_COMPAT_STATUS}" == "CUDA Driver OK" ]]; then
      echo
      echo "NOTE: Legacy NVIDIA Driver detected.  Compatibility mode ENABLED."
    else
      echo
      echo "ERROR: This container was built for NVIDIA Driver Release ${CUDA_DRIVER_VERSION%.*} or later, but"
      echo "       version ${DRIVER_VERSION} was detected and compatibility mode is UNAVAILABLE."
      echo
      echo "       [[${_CUDA_COMPAT_STATUS}]]"
      sleep 2
    fi
  fi
fi

if ! cat /proc/cpuinfo | grep flags | sort -u | grep avx >& /dev/null; then
  echo
  echo "ERROR: This container was built for CPUs supporting at least the AVX instruction set, but"
  echo "       the CPU detected was $(cat /proc/cpuinfo |grep "model name" | sed 's/^.*: //' | sort -u), which does not report"
  echo "       support for AVX.  An Illegal Instrution exception at runtime is likely to result."
  echo "       See https://en.wikipedia.org/wiki/Advanced_Vector_Extensions#CPUs_with_AVX ."
  sleep 2
fi


echo

cwd=$(pwd) && cd /home/abrk/catkin_ws && catkin build && cd $cwd

sleep 1

test -e /home/abrk/catkin_ws/devel/setup.bash && source /home/abrk/catkin_ws/devel/setup.bash

if [[ $# -eq 0 ]]; then
  exec "/bin/bash"
else
  exec "$@"
fi
