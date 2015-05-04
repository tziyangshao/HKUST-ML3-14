#!/bin/bash

rm change/Daniel_cmake
rm change/Daniel_fetcher

mv CMakeLists.txt change/Daniel_cmake
mv src/pc_fetcher.cpp change/Daniel_fetcher

mv change/Zyan_cmake CMakeLists.txt
mv change/Zyan_fetcher src/pc_fetcher.cpp
