#!/usr/bin/env bash
# Generate macOS .icns and Windows .ico from the source PNG icon.
# Requires: macOS (sips, iconutil), Python 3 + Pillow
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
SOURCE_PNG="$PROJECT_ROOT/resources/lumice_icon.png"
OUTPUT_ICNS="$PROJECT_ROOT/resources/lumice.icns"
OUTPUT_ICO="$PROJECT_ROOT/resources/lumice.ico"

# --- Dependency checks ---
if [[ "$(uname)" != "Darwin" ]]; then
  echo "Error: This script requires macOS (sips + iconutil)." >&2
  exit 1
fi

for cmd in sips iconutil python3; do
  if ! command -v "$cmd" &>/dev/null; then
    echo "Error: '$cmd' not found. Please install it." >&2
    exit 1
  fi
done

python3 -c "import PIL" 2>/dev/null || {
  echo "Error: Python Pillow not found. Install: pip3 install Pillow" >&2
  exit 1
}

if [[ ! -f "$SOURCE_PNG" ]]; then
  echo "Error: Source PNG not found at $SOURCE_PNG" >&2
  exit 1
fi

# --- Generate .icns ---
ICONSET_DIR=$(mktemp -d)/lumice.iconset
mkdir -p "$ICONSET_DIR"
trap 'rm -rf "$(dirname "$ICONSET_DIR")"' EXIT

echo "Generating .icns from $SOURCE_PNG ..."

# Standard macOS icon sizes: name → pixels
# icon_16x16.png      = 16
# icon_16x16@2x.png   = 32
# icon_32x32.png      = 32
# icon_32x32@2x.png   = 64
# icon_128x128.png    = 128
# icon_128x128@2x.png = 256
# icon_256x256.png    = 256
# icon_256x256@2x.png = 512
# icon_512x512.png    = 512
# (skip icon_512x512@2x.png = 1024, source is only 784px)

ICON_ENTRIES=(
  "icon_16x16.png:16"
  "icon_16x16@2x.png:32"
  "icon_32x32.png:32"
  "icon_32x32@2x.png:64"
  "icon_128x128.png:128"
  "icon_128x128@2x.png:256"
  "icon_256x256.png:256"
  "icon_256x256@2x.png:512"
  "icon_512x512.png:512"
)

for entry in "${ICON_ENTRIES[@]}"; do
  name="${entry%%:*}"
  size="${entry##*:}"
  sips -z "$size" "$size" "$SOURCE_PNG" --out "$ICONSET_DIR/$name" &>/dev/null
done

iconutil -c icns "$ICONSET_DIR" -o "$OUTPUT_ICNS"
echo "  -> $OUTPUT_ICNS"

# --- Generate .ico ---
echo "Generating .ico from $SOURCE_PNG ..."

python3 - "$SOURCE_PNG" "$OUTPUT_ICO" <<'PYEOF'
import sys
import struct
import io
from PIL import Image

source_path = sys.argv[1]
output_path = sys.argv[2]
sizes = [16, 24, 32, 48, 256]

img = Image.open(source_path).convert("RGBA")

# Build ICO file manually to guarantee all sizes are embedded.
# ICO format: 6-byte header + 16-byte entries + image data (PNG-encoded)
image_data_list = []
for s in sizes:
    resized = img.resize((s, s), Image.LANCZOS)
    buf = io.BytesIO()
    resized.save(buf, format="PNG")
    image_data_list.append(buf.getvalue())

num_images = len(sizes)
header = struct.pack('<HHH', 0, 1, num_images)  # reserved=0, type=1(icon), count

# Calculate offsets: header(6) + entries(16*N) + data
data_offset = 6 + 16 * num_images
entries = b''
for i, s in enumerate(sizes):
    w = 0 if s == 256 else s  # 0 means 256 in ICO format
    h = w
    data = image_data_list[i]
    entries += struct.pack('<BBBBHHII', w, h, 0, 0, 1, 32, len(data), data_offset)
    data_offset += len(data)

with open(output_path, 'wb') as f:
    f.write(header)
    f.write(entries)
    for data in image_data_list:
        f.write(data)

print(f"  Embedded sizes: {sizes} ({num_images} images)")
PYEOF

echo "  -> $OUTPUT_ICO"
echo "Done."
