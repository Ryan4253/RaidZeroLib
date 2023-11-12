cd -
git submodule update --init --recursive
rm -rf website
./docs/doxygen/m.css/documentation/doxygen.py --debug docs/doxygen/conf.py
mkdir website
cp -r docs/doxygen/html/* website
cp -r docs/images website
rm -rf docs/doxygen/latex docs/doxygen/xml docs/doxygen/html docs/doxygen/__pycache__
