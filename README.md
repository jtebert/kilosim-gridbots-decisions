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

## Debugging

To profile with `gprof` you need to run a different `cmake` command (from the build folder):

```
cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_CXX_FLAGS=-pg -DCMAKE_INSTALL_PREFIX=../ ..
```

This will generate a file called `gmon.out`. You can create an analysis with:

```
gprof gridbots_decisions gmon.out  > analysis.txt
```