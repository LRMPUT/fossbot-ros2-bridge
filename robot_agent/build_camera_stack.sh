#!/bin/bash
# Build the Raspberry Pi 5 camera stack: libpisp, libcamera, rpicam-apps.
#
# Why: Ubuntu 24.04 ships libcamera 0.2.0, whose only Raspberry Pi IPA is
# ipa_rpi_vc4.so (the Pi 4 ISP). The Pi 5 uses PiSP and needs ipa_rpi_pisp.so,
# and Ubuntu packages neither rpicam-apps nor picamera2. Without this the CSI
# only yields raw Bayer with no auto-exposure or white balance.
#
# Installs to /ws/opt/camera, which is a bind mount of the robot's agent
# directory, so the result survives recreating the container. The build and
# runtime dependencies are in the robot image (robot_agent/Dockerfile).
#
# Run on the robot, inside the agent container (setup_robot.sh --camera does
# this for you):
#   docker exec -it fb bash /ws/build_camera_stack.sh
#
# Takes roughly 15 minutes on a Pi 5.
set -euo pipefail

PREFIX=/ws/opt/camera
# Build under /ws too, not /tmp, so an interrupted build resumes after a
# container restart instead of starting over.
BUILD=/ws/opt/camera-build
# Deliberately not $(nproc): a four-core build can draw enough current to reset
# a battery-powered robot. Override with JOBS=4 on a solid supply.
JOBS="${JOBS:-2}"

mkdir -p "$BUILD" "$PREFIX"
export PKG_CONFIG_PATH="$PREFIX/lib/pkgconfig:${PKG_CONFIG_PATH:-}"
export LD_LIBRARY_PATH="$PREFIX/lib:${LD_LIBRARY_PATH:-}"

# --- libpisp: Raspberry Pi's PiSP front/back-end support library ------------
echo "=== building libpisp ==="
cd "$BUILD"
[ -d libpisp ] || git clone --depth 1 https://github.com/raspberrypi/libpisp.git
cd libpisp
[ -d build ] || meson setup build --prefix="$PREFIX" --buildtype=release -Dlibdir=lib >/dev/null
nice -n 10 ninja -C build -j "$JOBS"
ninja -C build install

# libpisp.pc exports "Cflags: -I${includedir}/libpisp", but libcamera includes
# the header as "libpisp/backend/backend.hpp", which needs the PARENT directory
# on the include path. Unpatched, the rpi/pisp pipeline fails with
#   fatal error: libpisp/backend/backend.hpp: No such file or directory
# The .pc is what is inconsistent with the header layout, so fix it there.
# (Passing -Dcpp_args to libcamera does not work: meson drops it on
# --reconfigure.)
PC="$PREFIX/lib/pkgconfig/libpisp.pc"
if [ -f "$PC" ] && ! grep -q 'I${includedir} ' "$PC"; then
  sed -i 's|^Cflags:.*|Cflags: -I${includedir} -I${includedir}/libpisp|' "$PC"
  echo "patched $PC include path"
fi

# --- libcamera: Raspberry Pi fork, which carries the rpi/pisp pipeline ------
echo "=== building libcamera (the long one) ==="
cd "$BUILD"
[ -d libcamera ] || git clone --depth 1 https://github.com/raspberrypi/libcamera.git
cd libcamera
# Meson caches dependency flags at configure time, so the .pc patch above must
# land before the first configure -- which it does. If you ever see the
# backend.hpp error with a build directory left over from an older script,
# delete $BUILD/libcamera/build and re-run.
[ -d build ] || meson setup build --prefix="$PREFIX" --buildtype=release -Dlibdir=lib \
  -Dpipelines=rpi/pisp,rpi/vc4 -Dipas=rpi/pisp,rpi/vc4 \
  -Dv4l2=true -Dgstreamer=disabled -Dtest=false -Dlc-compliance=disabled \
  -Ddocumentation=disabled -Dpycamera=disabled -Dcam=enabled -Dqcam=disabled \
  >/dev/null
nice -n 10 ninja -C build -j "$JOBS"
ninja -C build install

# --- rpicam-apps: provides rpicam-vid, which emits MJPEG on stdout -----------
echo "=== building rpicam-apps ==="
cd "$BUILD"
[ -d rpicam-apps ] || git clone --depth 1 https://github.com/raspberrypi/rpicam-apps.git
cd rpicam-apps
[ -d build ] || meson setup build --prefix="$PREFIX" --buildtype=release -Dlibdir=lib \
  -Denable_libav=disabled -Denable_drm=disabled -Denable_egl=disabled \
  -Denable_qt=disabled -Denable_opencv=disabled -Denable_tflite=disabled \
  -Denable_hailo=disabled \
  >/dev/null
nice -n 10 ninja -C build -j "$JOBS"
ninja -C build install

echo
echo "=== done. cameras visible to libcamera: ==="
# The IPA modules install one level below lib/libcamera.
export LIBCAMERA_IPA_MODULE_PATH="$PREFIX/lib/libcamera/ipa"
"$PREFIX/bin/cam" --list 2>&1 | grep -A5 "Available cameras" || true
echo "installed to $PREFIX"
