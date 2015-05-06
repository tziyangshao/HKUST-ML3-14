#!/bin/bash

rm change/Z_cmake
rm change/Z_fetcher

mv CMakeLists.txt change/Z_cmake
mv src/pc_fetcher.cpp change/Z_fetcher

cp change/D_cmake CMakeLists.txt
cp change/D_fetcher src/pc_fetcher.cpp

