#!/bin/bash

rm change/Zyan_cmake
rm change/Zyan_fetcher

mv CMakeLists.txt change/Zyan_cmake
mv src/pc_fetcher.cpp change/Zyan_fetcher

mv change/Daniel_cmake CMakeLists.txt
mv change/Daniel_fetcher src/pc_fetcher.cpp

