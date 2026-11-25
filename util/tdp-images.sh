#!/usr/bin/env bash
# Build and flash the TDP 2027 control-board firmware variants (TDP_2027_EXPERIMENTS.md, E6).
#
#   util/tdp-images.sh build              build shipped, nocap and clip into build/tdp-images/<commit>/
#   util/tdp-images.sh flash <variant>    flash one variant from the newest build (probe attached)
#   util/tdp-images.sh list               show built images
#
# Variants:
#   shipped  the normal image (planner cap on, direction-preserving FF budget)
#   nocap    feature tdp-ablation-nocap: no feasible-acceleration cap in position mode
#   clip     feature tdp-ablation-clip:  no cap, per-wheel FF current clipping
#
# Each build goes through `make control-board--control` (motor/kicker images and the
# image hash are embedded as usual). Run inside `nix develop`. The ablation images print
# "TDP ABLATION BUILD" at every boot. Reflash `shipped` when you are done.
#
# Building without the A-Team Wi-Fi credentials: NO_ATEAM_WIFI_CREDENTIALS=true util/tdp-images.sh build

set -euo pipefail
cd "$(dirname "$0")/.."

CHIP=STM32H723ZGTx
OUT_ROOT=build/tdp-images
ELF=control-board/target/thumbv7em-none-eabihf/release/control

declare -A FEATURES=([shipped]="" [nocap]="tdp-ablation-nocap" [clip]="tdp-ablation-clip")
VARIANTS=(shipped nocap clip)

features_flag() {
    local feats="$1"
    if [[ "${NO_ATEAM_WIFI_CREDENTIALS:-}" == "true" ]]; then
        feats="${feats:+$feats,}no-private-credentials"
    fi
    [[ -n "$feats" ]] && echo "--features=$feats" || echo ""
}

cmd_build() {
    local commit dirty out
    commit=$(git rev-parse --short HEAD)
    dirty=$(git diff --quiet HEAD -- control-board controls software-communication motor-controller kicker-board || echo "-dirty")
    out="$OUT_ROOT/$commit$dirty"
    mkdir -p "$out"
    for v in "${VARIANTS[@]}"; do
        echo "=== building $v"
        make control-board--control additional_control_cargo_flags="$(features_flag "${FEATURES[$v]}")"
        cp "$ELF" "$out/control-$v.elf"
        cp "$ELF.bin" "$out/control-$v.bin"
    done
    {
        echo "commit: $(git rev-parse HEAD)$dirty"
        echo "built:  $(date -Is)"
        for v in "${VARIANTS[@]}"; do echo "$v: features='${FEATURES[$v]}' sha256=$(sha256sum "$out/control-$v.elf" | cut -c1-16)"; done
    } > "$out/MANIFEST.txt"
    ln -sfn "$(basename "$out")" "$OUT_ROOT/latest"
    cat "$out/MANIFEST.txt"
    echo "images in $out (also $OUT_ROOT/latest)"
}

cmd_flash() {
    local v="${1:-}"
    [[ -n "${FEATURES[$v]+x}" ]] || { echo "usage: $0 flash shipped|nocap|clip" >&2; exit 2; }
    local elf="$OUT_ROOT/latest/control-$v.elf"
    [[ -f "$elf" ]] || { echo "$elf missing: run '$0 build' first" >&2; exit 1; }
    echo "flashing $elf"
    probe-rs download --chip "$CHIP" "$elf"
    probe-rs reset --chip "$CHIP"
    echo "flashed $v; check the boot log for 'TDP ABLATION BUILD' (absent on shipped)"
}

case "${1:-}" in
    build) cmd_build ;;
    flash) shift; cmd_flash "$@" ;;
    list) ls -l "$OUT_ROOT"/*/ 2>/dev/null || echo "no images yet" ;;
    *) sed -n '2,17p' "$0"; exit 2 ;;
esac
