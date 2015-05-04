#!/bin/bash

rm change/Z_cmake
rm change/Z_fetcher

mv CMakeLists.txt change/Z_cmake
mv src/pc_fetcher.cpp change/Z_fetcher

mv change/D_cmake CMakeLists.txt
mv change/D_fetcher src/pc_fetcher.cpp

