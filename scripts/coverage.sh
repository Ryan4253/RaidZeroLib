cd -
cd build
lcov --directory . --capture --output-file coverage.info
lcov --remove coverage.info '/usr/*' --output-file coverage.info
lcov --remove coverage.info '*/RaidZeroLib/build/*' --output-file coverage.info 
lcov --remove coverage.info '*/RaidZeroLib/include/okapi/*' --output-file coverage.info
#lcov --remove coverage.info '*/RaidZeroLib/test/include/*' --output-file coverage.info
#lcov --remove coverage.info '*/RaidZeroLib/test/src/*' --output-file coverage.info
#lcov --remove coverage.info '*/RaidZeroLib/test/test/*' --output-file coverage.info
lcov --list coverage.info