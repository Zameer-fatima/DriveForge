#!/usr/bin/env bash
# Downloads the free KITTI raw sample used by the recorded-data mode (2011-09-26, drive 0001: 108 frames, ~11 s).
# Download: ~460 MB (plus 0.3 MB of object annotations). On disk after extraction: ~320 MB (camera 02, lidar, GPS/IMU only).
#
# KITTI is licensed CC BY-NC-SA 3.0 (attribution, NON-COMMERCIAL, share-alike). It is not part of this repository and
# must not be redistributed with it. Cite: A. Geiger, P. Lenz, C. Stiller, R. Urtasun, "Vision meets Robotics: The KITTI
# Dataset", International Journal of Robotics Research (IJRR), 2013. https://www.cvlibs.net/datasets/kitti/
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
dest="$root/data/kitti"
tmp="$root/data/downloads"
base="https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_09_26_drive_0001"
drive="2011_09_26_drive_0001"

if [ -d "$dest" ] && [ -n "$(ls -A "$dest" 2>/dev/null)" ]; then
    echo "$dest already has data; delete it first to download again."
    exit 0
fi
command -v curl >/dev/null || { echo "curl is required"; exit 1; }
command -v unzip >/dev/null || { echo "unzip is required"; exit 1; }

mkdir -p "$tmp"
for name in "${drive}_sync.zip" "${drive}_tracklets.zip"; do
    remote="$(curl -sIL "$base/$name" | awk 'tolower($1)=="content-length:" {print $2}' | tr -d '\r' | tail -1)"
    have="$(stat -c %s "$tmp/$name" 2>/dev/null || echo 0)"
    if [ -n "$remote" ] && [ "$remote" = "$have" ]; then
        echo "$name already downloaded."
    else
        echo "Downloading $name ($((${remote:-0} / 1048576)) MB) ..."
        curl -L --fail --retry 3 -C - -o "$tmp/$name" "$base/$name"   # resumes a partial file
    fi
done

# Extract only what the app uses, into a new folder. unzip drops any absolute or ../ paths.
mkdir -p "$dest"
unzip -q "$tmp/${drive}_sync.zip" '*/oxts/*' '*/image_02/*' '*/velodyne_points/*' -d "$dest"
unzip -q "$tmp/${drive}_tracklets.zip" -d "$dest"

echo "Extracted to $dest. You can delete $tmp to free ~460 MB."
echo "Check it with:  ./build/driveforge --recorded-info \"$dest\""
