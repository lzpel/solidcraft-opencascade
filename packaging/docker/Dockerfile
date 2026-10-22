# syntax=docker/dockerfile:1
# SolidCraft in the browser as a container: the static site (the one packaging/web/package.sh
# zips) served by nginx on port 8080, as an unprivileged user. From the repository root:
#
#   docker build -f packaging/docker/Dockerfile -t solidcraft-web .
#   docker run --rm -p 8080:8080 solidcraft-web          then open http://localhost:8080/
#
# Or `docker compose up` with packaging/docker/compose.yaml. Saving designs needs HTTPS or
# localhost (the browser's private file storage), so put a TLS reverse proxy in front of it on a
# server; the site works under any URL path.
#
# The site is wasm, the same on every platform, so it is built once on the build machine's own
# platform even for a multi-platform image, inside cadrum's cross-compilation image (the
# OpenCASCADE kernel needs its wasi-sdk toolchain). CI builds it outside this file (with its
# cache and git commit for Help › About) and passes it in as the `site` stage:
# --build-context site=dist/web.

ARG TRUNK_VERSION=0.21.14

FROM --platform=$BUILDPLATFORM ghcr.io/lzpel/cross-wasm32-unknown-unknown AS build
ARG TRUNK_VERSION
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN curl -fsSL "https://github.com/trunk-rs/trunk/releases/download/v${TRUNK_VERSION}/trunk-$(uname -m)-unknown-linux-gnu.tar.gz" \
    | tar -xz -C /usr/local/bin trunk
WORKDIR /src
COPY . .
WORKDIR /src/apps/solidcraft-web
# The image's cargo targets wasm32 by default; trunk asks for wasm32 itself, and the post-build
# hook (`cargo xtask web-offline`, which writes sw.js) must run on the container's own target.
ENV CARGO_BUILD_TARGET=x86_64-unknown-linux-gnu
# Trunk downloads the wasm-bindgen and wasm-opt its build needs.
RUN --mount=type=cache,target=/usr/local/cargo/registry \
    --mount=type=cache,target=/src/target \
    trunk build --release

FROM scratch AS site
COPY --from=build /src/dist/web/ /

FROM nginxinc/nginx-unprivileged:1.29-alpine
COPY packaging/docker/nginx.conf /etc/nginx/conf.d/default.conf
COPY LICENSE-MIT LICENSE-APACHE NOTICE THIRD-PARTY-LICENSES.txt /usr/share/doc/solidcraft/
COPY --from=site / /srv/solidcraft/
EXPOSE 8080
HEALTHCHECK --interval=30s --timeout=5s CMD wget -q --spider http://127.0.0.1:8080/ || exit 1
