#!/usr/bin/env bash
# One-time base-computer setup. Runtime never installs packages.
set -eo pipefail
source /opt/ros/humble/setup.bash
if [[ -f "${HOME}/ros2_ws/install/setup.bash" ]]; then
  source "${HOME}/ros2_ws/install/setup.bash"
fi

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
venv="${root}/.venv-letter"
deps="${root}/.letter-deps"
wheelhouse="${root}/wheels/letter"

if [[ -d "${deps}" ]] && \
  PYTHONPATH="${deps}${PYTHONPATH:+:${PYTHONPATH}}" python3 -c \
    'import cv2, numpy, rclpy; assert numpy.__version__.startswith("1.")' 2>/dev/null; then
  PYTHONPATH="${deps}${PYTHONPATH:+:${PYTHONPATH}}" python3 -c \
    'import cv2, numpy, rclpy; print("mode=target", cv2.__version__, numpy.__version__, "rclpy-ok")'
  exit 0
fi

install_vision_wheels() {
  local python="$1"
  shift
  if compgen -G "${wheelhouse}/*.whl" >/dev/null; then
    "${python}" -m pip install --disable-pip-version-check --upgrade \
      --no-index --find-links "${wheelhouse}" "$@" \
      'numpy==1.26.4' 'opencv-python-headless==4.10.0.84'
  else
    "${python}" -m pip install --disable-pip-version-check --upgrade "$@" \
      'numpy>=1.24,<2' 'opencv-python-headless>=4.8,<4.11'
  fi
}

if python3 -m venv --system-site-packages "${venv}" 2>/dev/null; then
  install_vision_wheels "${venv}/bin/python"
  "${venv}/bin/python" -c \
    'import cv2, numpy, rclpy; print("mode=venv", cv2.__version__, numpy.__version__, "rclpy-ok")'
else
  # Ubuntu images without python3-venv can still keep all wheels isolated in
  # the project. PYTHONPATH puts these compatible wheels before system NumPy.
  mkdir -p "${deps}"
  install_vision_wheels python3 --target "${deps}"
  PYTHONPATH="${deps}${PYTHONPATH:+:${PYTHONPATH}}" python3 -c \
    'import cv2, numpy, rclpy; print("mode=target", cv2.__version__, numpy.__version__, "rclpy-ok")'
fi
