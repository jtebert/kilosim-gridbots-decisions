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

## Batch run/management

To run a whole batch, use `manage.py`.

First, set up a Python virtual environment the first time you're using this:
```shell
python3 -m venv venv
pip install -r requirements.txt
```

Then activate it when you want to run this:
```shell
source venv/bin/activate
```

You can now run the following options for generating parameter sweeps and running them.

- `python manage.py generate SOURCE_YAML`: Generate all of the specific configuration files from a generator YAML file (see distributed/gen_config.yml for an example). This will create all of the data directories *that don't already exist* and place the config files in them. It will also generate `data_dirs.txt` in the current directory containing the paths to all of the new folders/configs that were generated. (This will *not* include any folders that are skipped because they already exist.)
- `python manage.py split NUM`: Split `data_dirs.txt` into multiple files (`data_dirs_split_NUM.txt`) to use on different system threads.
- `python manage.py run EXEC_FILE NUM`: Run all of the experiments listed in `data_dirs_split_NUM.txt`. This will pass the config file found in the directory to the `EXEC_FILE` to run. For example: `python manage.py distributed/main.py 1`.