#!/usr/bin/env bash
# ensure that you have libheif
# sudo apt install libheif-examples
for file in *.heic; do heif-convert $file ${file/%.heic/.jpg}; done
for file in *.HEIC; do heif-convert $file ${file/%.HEIC/.jpg}; done

