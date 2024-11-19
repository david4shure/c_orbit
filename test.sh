make clean
cmake .
cmake --build .
ctest --output-on-failure -V
