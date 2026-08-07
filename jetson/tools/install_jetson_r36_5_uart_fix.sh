#!/usr/bin/env bash
set -euo pipefail

if [[ $EUID -ne 0 ]]; then
    echo "Run this installer with sudo." >&2
    exit 1
fi

DTB_NAME="kernel_tegra234-p3768-0000+p3767-0005-nv-super.dtb"
TARGETS=("/boot/$DTB_NAME" "/boot/dtb/$DTB_NAME")
EXTLINUX="/boot/extlinux/extlinux.conf"
WORKDIR="$(mktemp -d)"
trap 'rm -rf "$WORKDIR"' EXIT
SOURCE_DTB="${TARGETS[1]}"
SOURCE_DTS="$WORKDIR/source.dts"
PATCHED="$WORKDIR/uart1-pio.dtb"

for target in "${TARGETS[@]}"; do
    if [[ ! -f "$target" ]]; then
        echo "Expected active DTB not found: $target" >&2
        exit 1
    fi
done

if [[ ! -f "$EXTLINUX" ]]; then
    echo "Boot configuration not found: $EXTLINUX" >&2
    exit 1
fi

# L4T R36.5 defines UART1 DMA channels without the matching IOMMU property.
# Remove only those DMA properties to force the serial-tegra driver into PIO
# mode, which avoids null-byte corruption on the 40-pin-header UART.
dtc -I dtb -O dts -o "$SOURCE_DTS" "$SOURCE_DTB"
sed -i '/serial@3100000 {/,/^[[:space:]]*};/ {
  /^[[:space:]]*dmas = /d
  /^[[:space:]]*dma-names = /d
}' "$SOURCE_DTS"
dtc -I dts -O dtb -o "$PATCHED" "$SOURCE_DTS"

if fdtget -p "$PATCHED" /bus@0/serial@3100000 | grep -qxE 'dmas|dma-names'; then
    echo "UART1 DMA properties are still present in generated DTB." >&2
    exit 1
fi

stamp="$(date +%Y%m%d_%H%M%S)"
for target in "${TARGETS[@]}"; do
    cp -a "$target" "$target.before-uart-fix-$stamp"
    install -m 0644 "$PATCHED" "$target"
    echo "Installed UART fix: $target"
done

cp -a "$EXTLINUX" "$EXTLINUX.before-uart-fix-$stamp"
if grep -qE '^[[:space:]]*FDT[[:space:]]+' "$EXTLINUX"; then
    sed -i -E "s|^[[:space:]]*FDT[[:space:]]+.*|      FDT /boot/dtb/$DTB_NAME|" "$EXTLINUX"
else
    sed -i "/^[[:space:]]*LINUX[[:space:]]\+\/boot\/Image[[:space:]]*$/a\\      FDT /boot/dtb/$DTB_NAME" "$EXTLINUX"
fi
echo "Selected patched DTB in $EXTLINUX"

sync
echo "UART1 is configured for PIO mode. Reboot the Jetson to activate it."
