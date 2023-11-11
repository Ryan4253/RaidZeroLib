cd -
cd build
valgrind --tool=memcheck --leak-check=full --leak-resolution=med --show-leak-kinds=all --undef-value-errors=yes --track-origins=yes  --error-exitcode=1 --show-reachable=no ./RaidZeroLib
