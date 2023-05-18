Note to install ibex-lib

```bash 
git clone -b develop https://github.com/ibex-team/ibex-lib.git
cd ibex-lib
mkdir -p build && cd build
# or -DCMAKE_INSTALL_PREFIX=${HOME}/ibex
cmake -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_BUILD_TYPE=Relase -DCMAKE_CXX_FLAGS="-fPIC" -DCMAKE_C_FLAGS="-fPIC" ..
make && make check && make install
```