## How initially created

```shell
mkdir submodules
cd submodules
git submodule add git@github.com:jtebert/kilosim-gridbots.git
git submodule update --init --recursive
cd ..
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_INSTALL_PREFIX=../ ..
make
```

## Use

```shell
make clean  # Remove existing
make  # Compile
make install  # Install to bin/ directory
```

From project root directory, run:
```shell
./bin/gridbots_decisions
```