#!/bin/bash
# Build the Raspberry Pi 5 camera stack on the robot.
#
# Why this is needed: Ubuntu 24.04 ships libcamera 0.2.0, whose only Raspberry
# Pi IPA is ipa_rpi_vc4.so -- the Pi 4 ISP. The Pi 5 uses PiSP, needs
# ipa_rpi_pisp.so, and Ubuntu has no rpicam-apps or picamera2 at all. Without
# this the CSI only yields raw Bayer with no AE/AWB.
#
# Installs to a prefix under /ws (which is /home/admin/dev on the robot's real
# filesystem) so the result survives `docker rm` of the fb container and does
# not add half an hour to every image rebuild.
#
# Run inside the fb container:
#   sudo docker exec -it fb bash /ws/build_camera_stack.sh
set -euo pipefail

PREFIX=/ws/opt/camera
# Build under /ws too, not /tmp: the build survives a container restart, so an
# interrupted run resumes instead of starting over.
BUILD=/ws/opt/camera-build
# Deliberately not $(nproc). A full four-core ninja on a Pi 5 drew enough
# current to drop this robot off the network mid-build; leaving a core idle
# keeps it responsive and the supply happier. Override with JOBS=4.
JOBS="${JOBS:-2}"

echo "=== installing build dependencies ==="
export DEBIAN_FRONTEND=noninteractive
apt-get update -qq
apt-get install -y --no-install-recommends \
  git cmake meson ninja-build pkg-config build-essential \
  python3-yaml python3-ply python3-jinja2 python3-pip \
  libyaml-dev libgnutls28-dev openssl \
  libdrm-dev libjpeg-dev libtiff-dev libpng-dev libexif-dev \
  libevent-dev libboost-program-options-dev \
  nlohmann-json3-dev libssl-dev

mkdir -p "$BUILD" "$PREFIX"
export PKG_CONFIG_PATH="$PREFIX/lib/aarch64-linux-gnu/pkgconfig:$PREFIX/lib/pkgconfig:${PKG_CONFIG_PATH:-}"
export LD_LIBRARY_PATH="$PREFIX/lib/aarch64-linux-gnu:$PREFIX/lib:${LD_LIBRARY_PATH:-}"

# --- libpisp: Raspberry Pi's PiSP front/back-end support library -------------
echo "=== building libpisp ==="
cd "$BUILD"
[ -d libpisp ] || git clone --depth 1 https://github.com/raspberrypi/libpisp.git
cd libpisp
[ -d build ] || meson setup build --prefix="$PREFIX" --buildtype=release -Dlibdir=lib >/dev/null
nice -n 10 ninja -C build -j "$JOBS"
ninja -C build install

# libpisp.pc says "Cflags: -I${includedir}/libpisp", but every consumer --
# libcamera included -- includes the header as "libpisp/backend/backend.hpp",
# which needs the PARENT directory on the include path. Without this, building
# the rpi/pisp pipeline dies with:
#   fatal error: libpisp/backend/backend.hpp: No such file or directory
# Fixing the .pc is the right place: it is the .pc that is inconsistent with
# how the headers are laid out and used. (-Dcpp_args does not work here --
# meson silently drops it on --reconfigure.)
PC="$PREFIX/lib/pkgconfig/libpisp.pc"
if [ -f "$PC" ] && ! grep -q 'I${includedir} ' "$PC"; then
  sed -i 's|^Cflags:.*|Cflags: -I${includedir} -I${includedir}/libpisp|' "$PC"
  echo "patched $PC include path"
fi

# --- libcamera: Raspberry Pi fork, which carries the rpi/pisp pipeline -------
echo "=== building libcamera (this is the long one) ==="
cd "$BUILD"
[ -d libcamera ] || git clone --depth 1 https://github.com/raspberrypi/libcamera.git
cd libcamera
# -I$PREFIX/include is not optional and pkg-config will not supply it:
# libpisp.pc exports "-I${includedir}/libpisp", but libcamera includes the
# header as "libpisp/backend/backend.hpp", so it needs the PARENT directory on
# the include path. Without this the rpi/pisp pipeline fails to compile with
# "fatal error: libpisp/backend/backend.hpp: No such file or directory".
LIBCAMERA_ARGS=(--prefix="$PREFIX" --buildtype=release -Dlibdir=lib
  -Dcpp_args="-I$PREFIX/include"
  -Dpipelines=rpi/pisp,rpi/vc4
  -Dipas=rpi/pisp,rpi/vc4
  -Dv4l2=true -Dgstreamer=disabled -Dtest=false -Dlc-compliance=disabled
  -Ddocumentation=disabled -Dpycamera=disabled -Dcam=enabled -Dqcam=disabled)
if [ -d build ]; then
  meson setup --reconfigure build "${LIBCAMERA_ARGS[@]}" >/dev/null
else
  meson setup build "${LIBCAMERA_ARGS[@]}" >/dev/null
fi
nice -n 10 ninja -C build -j "$JOBS"
ninja -C build install

# --- rpicam-apps: gives rpicam-vid, which can emit MJPEG on stdout -----------
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
echo "=== done. verifying ==="
export LIBCAMERA_IPA_MODULE_PATH="$PREFIX/lib/libcamera"
"$PREFIX/bin/cam" --list 2>&1 | head -20
echo
echo "installed to $PREFIX"
echo "to use:  export LD_LIBRARY_PATH=$PREFIX/lib; export PATH=$PREFIX/bin:\$PATH"
