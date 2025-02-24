#!/usr/bin/env bash
# ensure that you have libheif
# sudo apt install libheif-examples
for file in *.heic; do heif-convert $file ${file/%.heic/.jpg}; done
for file in *.HEIC; do heif-convert $file ${file/%.HEIC/.jpg}; done

# if flag --hdr passed, don't delete files with hdrgainmap in name afterwards
if [ "$1" == "--hdr" ]; then
    echo "HDR flag passed, not deleting hdrgainmap files"
else
    echo "Deleting hdrgainmap files"
    rm ./*hdrgainmap*
fi
