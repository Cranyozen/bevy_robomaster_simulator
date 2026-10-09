#!/usr/bin/env bash
# verify_payload.sh — hard acceptance gate for the runtime image payload.
#
# Runs INSIDE the runtime stage at build time. It exists as a file rather than an
# inline `RUN` because the shell constructs involved (case/;;, loops) are
# mangled by Dockerfile line-continuation parsing — `;;` is treated as the start
# of a Dockerfile comment.
#
# Fails (non-zero) if ANY of the following is false:
#   1. the dynamic-linking payloads were staged (libbevy_dylib-*.so, libstd-*.so);
#   2. the binary is a real, non-trivial ELF executable — this is the
#      ANTI-FALSE-PASS check: an earlier revision shipped a 450 KB `fn main() {}`
#      stub that satisfied `test -x` and a clean `ldd`;
#   3. every hash-suffixed NEEDED soname in the binary is a file we actually
#      shipped in $APP_LIB (self-consistency — deliberately NOT compared against
#      any host toolchain hash, since those legitimately differ);
#   4. `ldd` resolves every library (no "not found").
set -euo pipefail

APP_BIN="${APP_BIN:-/app/daedalus}"
APP_LIB="${APP_LIB:-/app/lib}"

say() { printf '[verify_payload] %s\n' "$*"; }
die() { printf '[verify_payload] FAIL: %s\n' "$*" >&2; exit 1; }

# --- 1. staged payload --------------------------------------------------------
[[ -x "$APP_BIN" ]] || die "$APP_BIN missing or not executable"

shopt -s nullglob
bevy_libs=("$APP_LIB"/libbevy_dylib-*.so)
std_libs=("$APP_LIB"/libstd-*.so)
shopt -u nullglob

(( ${#bevy_libs[@]} > 0 )) || die "no libbevy_dylib-*.so staged in $APP_LIB"
(( ${#std_libs[@]} > 0 ))  || die "no libstd-*.so staged in $APP_LIB"
say "staged bevy: ${bevy_libs[*]##*/}"
say "staged std:  ${std_libs[*]##*/}"

# --- 2. anti-false-pass: real ELF, non-trivial size ---------------------------
# `file(1)` is deliberately NOT required: the runtime image is intentionally lean
# and does not ship it. The ELF magic (0x7f 'E' 'L' 'F') plus the 64-bit/LE
# marker in e_ident are checked directly, which needs no extra package.
magic="$(head -c 5 "$APP_BIN" | od -An -tx1 | tr -d ' \n')"
[[ "$magic" == "7f454c4602" ]] \
    || die "$APP_BIN is not an ELF 64-bit little-endian executable (magic=$magic)"
say "ELF magic ok ($magic)"

BIN_BYTES="$(stat -c %s "$APP_BIN")"
say "binary size: ${BIN_BYTES} bytes"
# The real dyn-linked app is ~45-47 MB; the accidental stub was 450,720 bytes.
(( BIN_BYTES > 5000000 )) || die "binary is stub-sized (${BIN_BYTES} bytes) — build did not compile the real crate"

# --- 3. NEEDED self-consistency ----------------------------------------------
# The binary must genuinely depend on the bevy dylib (a stub does not).
readelf -d "$APP_BIN" | grep -q 'libbevy_dylib-' \
    || die "binary has no libbevy_dylib-* NEEDED entry — not a dynamic_linking build"

mapfile -t needed < <(readelf -d "$APP_BIN" | sed -n 's/.*NEEDED.*\[\(.*\)\].*/\1/p')
say "NEEDED: ${needed[*]}"
for so in "${needed[@]}"; do
    case "$so" in
        libbevy_dylib-*.so|libstd-*.so)
            [[ -f "$APP_LIB/$so" ]] || die "NEEDED $so is not present in $APP_LIB"
            say "NEEDED $so -> present in $APP_LIB"
            ;;
    esac
done

# --- 4. full linkage ----------------------------------------------------------
ldd "$APP_BIN" | tee /tmp/ldd.txt
if grep -q 'not found' /tmp/ldd.txt; then
    die "unresolved shared library (see 'not found' above)"
fi
say "all shared libraries resolve"

say "PASS (binary ${BIN_BYTES} bytes)"
