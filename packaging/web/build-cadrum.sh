#!/usr/bin/env sh
# Build the web app, inside cadrum's cross-compilation image (see packaging/docker/Dockerfile.web):
# OpenCASCADE, the default kernel backend, builds for wasm32 only with its wasi-sdk toolchain, so
# `trunk build` on the host does not work. Run from any directory; the output is dist/web.
set -eu
# Git Bash on Windows would turn the container paths into host paths otherwise.
export MSYS_NO_PATHCONV=1
cd "$(dirname "$0")/../.."
docker build -t solidcraft-web-builder -f packaging/docker/Dockerfile.web packaging/docker
# The image's cargo targets wasm32 by default; trunk asks for wasm32 itself, and the post-build
# hook (`cargo xtask web-offline`, which lists the build's files for the service worker) must run
# on the container's own host target.
docker run --rm -v "$PWD":/work -w /work/apps/solidcraft-web \
    -e CARGO_TARGET_DIR=/work/target/wasm-docker \
    -e CARGO_BUILD_TARGET=x86_64-unknown-linux-gnu \
    solidcraft-web-builder trunk build --release
