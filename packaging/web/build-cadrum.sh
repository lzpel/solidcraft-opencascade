#!/usr/bin/env sh
# Build the web app with the OpenCASCADE kernel backend, inside cadrum's cross-compilation
# image (see packaging/docker/Dockerfile.web). Run from the repository root; the output is
# dist/web, as `trunk build --release` leaves it for the truck backend.
set -eu
cd "$(dirname "$0")/../.."
docker build -t solidcraft-web-builder -f packaging/docker/Dockerfile.web packaging/docker
docker run --rm -v "$PWD":/work -w /work/apps/solidcraft-web \
    -e CARGO_TARGET_DIR=/work/target/wasm-docker \
    solidcraft-web-builder trunk build --release --features cadrum
