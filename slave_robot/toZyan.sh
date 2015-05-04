#!/bin/bash

rm change/D_cmake
rm change/D_fetcher

mv CMakeLists.txt change/Z_cmake
mv src/pc_fetcher.cpp change/Z_fetcher

mv change/Z_cmake CMakeLists.txt
mv change/Z_fetcher src/pc_fetcher.cpp

