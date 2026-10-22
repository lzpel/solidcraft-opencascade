#!/usr/bin/env sh
# Build the web app with the OpenCASCADE kernel backend, inside cadrum's cross-compilation
# image (see packaging/docker/Dockerfile.web). Run from the repository root; the output is
# dist/web, as `trunk build --release` leaves it for the truck backend.
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
    solidcraft-web-builder trunk build --release --features cadrum
