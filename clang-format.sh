find include/RaidZeroLib/ -iname *.hpp | xargs clang-format -i -style=file
find src/ -iname *.cpp | xargs clang-format -i -style=file
find test/ -iname *.hpp | xargs clang-format -i -style=file
find test/ -iname *.cpp | xargs clang-format -i -style=file
