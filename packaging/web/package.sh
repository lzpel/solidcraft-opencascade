#!/usr/bin/env bash
# Build the browser version and zip it:  $DIST/solidcraft-web-<version>.zip
#
# Usage: packaging/web/package.sh [--skip-build]
#
# Needs: docker (the build runs inside cadrum's image, packaging/web/build-cadrum.sh). The zip
# holds a self-contained static site
# in solidcraft-web-<version>/ that works from any URL path (public_url = "./" in Trunk.toml).
# Saving designs needs HTTPS or localhost (the browser's private file storage).
# Adapted from PhotoCraft's packaging (storytold/photocraft, Apache-2.0; see ATTRIBUTION.md).
set -euo pipefail
# shellcheck source=../env.sh
. "$(dirname "${BASH_SOURCE[0]}")/../env.sh"

if [ "${1:-}" != "--skip-build" ]; then
  command -v docker >/dev/null || { echo "error: docker not found (the web app is built inside cadrum's image)" >&2; exit 1; }
  sh "$ROOT/packaging/web/build-cadrum.sh"
fi

SITE="$ROOT/dist/web"
[ -f "$SITE/index.html" ] || { echo "error: $SITE/index.html missing; run without --skip-build" >&2; exit 1; }
# Paths must be relative so the site works under any prefix.
if grep -Eq '(src|href)="/[^/]' "$SITE/index.html"; then
  echo "error: $SITE/index.html has root-absolute URLs; it would break when served from a sub-path" >&2
  exit 1
fi

NAME="solidcraft-web-$VERSION"
WORK="$CARGO_TARGET_DIR/web-package"
rm -rf "$WORK"
mkdir -p "$WORK/$NAME"
cp -R "$SITE/." "$WORK/$NAME/"
copy_docs "$WORK/$NAME"
rm -f "$DIST/$NAME.zip"
(cd "$WORK" && zip -qr9 "$DIST/$NAME.zip" "$NAME")
echo "wrote $DIST/$NAME.zip"
ls -lh "$DIST/$NAME.zip"
