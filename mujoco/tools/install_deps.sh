#!/usr/bin/env bash
# Installs the k1_mujoco_sim build prerequisites that CMake does not fetch itself:
#   - MuJoCo prebuilt release (Linux library or macOS universal framework)
#   - Fast-CDR / foonathan_memory_vendor / Fast-DDS (built from source)
#   - Fast-DDS-Gen (fastddsgen), the IDL code generator
#
# fastddsgen is part of the standard set, not an extra: CMake now runs it at
# build time to generate the Fast-DDS types into the build dir, so they are no
# longer committed to the repo. A Java 11+ runtime is therefore a hard
# requirement of this script and of the sim build, not an optional add-on.
#
# This is normally run INSIDE the docker image build (docker/Dockerfile) with
# PREFIX=/opt/k1sim-deps — see docker/k1sim.sh for the supported workflow.
# Running it directly on a host is a fallback for native development only.
#
# Usage:
#   tools/install_deps.sh                  # installs everything, fastddsgen included
#   tools/install_deps.sh --with-fastddsgen  # accepted for backwards compatibility; does nothing
#
# (Unknown arguments are ignored rather than fatal, so older callers such as
# docker/Dockerfile that still pass --with-fastddsgen keep working unchanged.)
#
# Version pins (rationale in docs/K1_MUJOCO_SETUP.md):
#   Fast-DDS v2.13.6 matches the minor version bundled with Booster's SDK (2.13.1),
#   eliminating DDS interop drift against NUbots_K1's B1LocoClient.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)" # mujoco/
DEPS="${DEPS:-$HERE/.deps}"
PREFIX="${PREFIX:-$DEPS/install}"
SRC="${SRC:-$DEPS/src}"
PLATFORM="$(uname -s)"
ARCH="$(uname -m)"
case "$PLATFORM" in
    Darwin) JOBS="${JOBS:-$(sysctl -n hw.logicalcpu)}" ;;
    Linux) JOBS="${JOBS:-$(nproc)}" ;;
    *) echo "error: unsupported platform: $PLATFORM" >&2; exit 1 ;;
esac

# Check before downloading/building anything. On macOS the system java command
# can be a stub; JAVA_HOME selects a Homebrew JDK without a system-wide install.
JAVA="${JAVA_HOME:+$JAVA_HOME/bin/}java"
if ! "$JAVA" -version > /dev/null 2>&1; then
    echo "error: a Java 11+ JDK is required to build and run fastddsgen." >&2
    echo "On macOS: brew install openjdk@17; export JAVA_HOME=\$(brew --prefix openjdk@17)/libexec/openjdk.jdk/Contents/Home" >&2
    exit 1
fi
JAVA_MAJOR="$("$JAVA" -version 2>&1 | sed -n 's/.*version "\([0-9]*\).*/\1/p' | head -1)"
if [ ! -f "$PREFIX/share/fastddsgen/java/fastddsgen.jar" ] && \
    { [ "${JAVA_MAJOR:-0}" -lt 11 ] || [ "${JAVA_MAJOR:-0}" -gt 19 ]; }; then
    echo "error: pinned fastddsgen uses Gradle 7.6, which needs JDK 11–19 to build (17 recommended)." >&2
    echo "Select it with JAVA_HOME; newer Java runtimes can run the generated jar afterwards." >&2
    exit 1
fi

MUJOCO_VERSION=3.10.0
FOONATHAN_TAG=v1.3.1
FASTCDR_TAG=v2.1.3
FASTDDS_TAG=v2.13.6
FASTDDSGEN_TAG=v3.2.1

mkdir -p "$PREFIX" "$SRC"

# --- MuJoCo prebuilt ---------------------------------------------------------
if [ "$PLATFORM" = Darwin ]; then
    MUJOCO_LIB="libmujoco.dylib"
else
    MUJOCO_LIB="libmujoco.so"
fi
if [ ! -f "$PREFIX/mujoco-$MUJOCO_VERSION/lib/$MUJOCO_LIB" ]; then
    echo "== MuJoCo $MUJOCO_VERSION (prebuilt)"
    RELEASE="https://github.com/google-deepmind/mujoco/releases/download/$MUJOCO_VERSION"
    if [ "$PLATFORM" = Darwin ]; then
        curl -fL "$RELEASE/mujoco-$MUJOCO_VERSION-macos-universal2.dmg" -o "$SRC/mujoco.dmg"
        MOUNT="$(mktemp -d)"
        trap 'hdiutil detach "$MOUNT" > /dev/null 2>&1 || true; rmdir "$MOUNT"' EXIT
        hdiutil attach -nobrowse -readonly -mountpoint "$MOUNT" "$SRC/mujoco.dmg"
        MJ_PREFIX="$PREFIX/mujoco-$MUJOCO_VERSION"
        mkdir -p "$MJ_PREFIX/include" "$MJ_PREFIX/lib"
        cp -R "$MOUNT/mujoco.framework" "$MJ_PREFIX/"
        # Keep the signed framework intact (its install name is @rpath/mujoco.framework/...),
        # while exposing the same include/lib layout used by the Linux build.
        ln -sfn ../mujoco.framework/Headers "$MJ_PREFIX/include/mujoco"
        ln -sfn "../mujoco.framework/Versions/A/libmujoco.$MUJOCO_VERSION.dylib" "$MJ_PREFIX/lib/libmujoco.dylib"
        hdiutil detach "$MOUNT"
        rmdir "$MOUNT"
        trap - EXIT
    else
        case "$ARCH" in
            x86_64) MJ_ARCH=x86_64 ;;
            aarch64|arm64) MJ_ARCH=aarch64 ;;
            *) echo "error: unsupported Linux architecture: $ARCH" >&2; exit 1 ;;
        esac
        curl -fL "$RELEASE/mujoco-$MUJOCO_VERSION-linux-$MJ_ARCH.tar.gz" -o "$SRC/mujoco.tar.gz"
        tar -xzf "$SRC/mujoco.tar.gz" -C "$PREFIX"
    fi
else
    echo "== MuJoCo $MUJOCO_VERSION (cached)"
fi

# --- CMake source builds -----------------------------------------------------
build() { # build <name> <repo> <tag> [cmake args...]
    local name=$1 repo=$2 tag=$3
    shift 3
    local stamp="$PREFIX/.stamp-$name-$tag"
    if [ "$PLATFORM" = Darwin ] && [ "$name" = fastdds ]; then
        stamp="$stamp-macos-affinity-v1"
    fi
    if [ -f "$stamp" ]; then
        echo "== $name $tag (cached)"
        return
    fi
    echo "== $name $tag"
    rm -rf "$SRC/$name"
    git clone --depth 1 --branch "$tag" "$repo" "$SRC/$name"
    if [ "$PLATFORM" = Darwin ] && [ "$name" = fastdds ]; then
        cmake -DFASTDDS_SOURCE="$SRC/$name" -P "$HERE/cmake/PatchFastDDS.cmake"
    fi
    cmake -S "$SRC/$name" -B "$SRC/$name/build" -GNinja \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="$PREFIX" \
        -DCMAKE_PREFIX_PATH="$PREFIX" \
        -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
        "$@"
    cmake --build "$SRC/$name/build" -j"$JOBS"
    cmake --install "$SRC/$name/build"
    touch "$stamp"
}

build foonathan_memory_vendor https://github.com/eProsima/foonathan_memory_vendor.git "$FOONATHAN_TAG"
build fastcdr https://github.com/eProsima/Fast-CDR.git "$FASTCDR_TAG" -DBUILD_SHARED_LIBS=ON
build fastdds https://github.com/eProsima/Fast-DDS.git "$FASTDDS_TAG" \
    -DBUILD_SHARED_LIBS=ON \
    -DTHIRDPARTY_Asio=FORCE \
    -DTHIRDPARTY_TinyXML2=FORCE \
    -DCOMPILE_EXAMPLES=OFF \
    -DCOMPILE_TOOLS=OFF \
    -DSECURITY=OFF \
    -DNO_TLS=ON # the sim uses no TLS transport, and the vendored Asio does not compile against OpenSSL 4 (Homebrew's default)

# --- fastddsgen (required, CMake generates the DDS types at build time) ------
if [ ! -f "$PREFIX/share/fastddsgen/java/fastddsgen.jar" ]; then
    echo "== Fast-DDS-Gen $FASTDDSGEN_TAG"
    rm -rf "$SRC/fastddsgen"
    git clone --recursive --depth 1 --branch "$FASTDDSGEN_TAG" \
        https://github.com/eProsima/Fast-DDS-Gen.git "$SRC/fastddsgen"
    (cd "$SRC/fastddsgen" && ./gradlew assemble)
    mkdir -p "$PREFIX/share/fastddsgen/java" "$PREFIX/bin"
    cp "$SRC/fastddsgen/share/fastddsgen/java/fastddsgen.jar" "$PREFIX/share/fastddsgen/java/"
    cp "$SRC/fastddsgen/scripts/fastddsgen" "$PREFIX/bin/"
    chmod +x "$PREFIX/bin/fastddsgen"
else
    echo "== Fast-DDS-Gen (cached)"
fi

echo
echo "Dependencies installed to $PREFIX"
echo "Configure the sim with:  cmake -S \"$HERE\" -B \"$HERE/build\" -GNinja"
