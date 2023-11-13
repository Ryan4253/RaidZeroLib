cd -
cppcheck --enable=all --suppress=missingInclude --suppress=unusedFunction --error-exitcode=1 -I include/RaidZeroLib src/RaidZeroLib