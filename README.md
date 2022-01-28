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

## Running on multiple computers

This is the process for splitting a batch to run across multiple computers

- On first computer (lab):
  - Make and install (from inside `build`): `make clean`, `make install`
  - Generate folders: `python manage.py generate post_decision_gen_config_2.yml -o post_data_dirs.txt`
  - Split by *total* number of threads, with shuffle: `python manage.py split 16 -i post_data_dirs.txt --shuffle`
  - Commit & push, including the split files
  - Set up tmux session: `tmux` (To detach: `Ctrl`+`b`, `d`. To reattach: `tmux attach`)
  - Run the threads for that machine only: `python manage.py run "./bin/gridbots_decisions" 0 1 2 3 -i post_data_dirs_split_#.txt`
- On other computers:
  - If relevant, activate Python virtual environment
  - Pull code: `git pull`
  - Build and install any updates: `make clean`, `make install`
  - Change config base folder name in `post_decision_gen_config_2.yml`
  - Generate folders: `python manage.py generate post_decision_gen_config_2.yml -o post_data_dirs.txt`
  - Rename folders:
    - For lab computer: `python manage.py rename "/media/jtebert/home-data/hybrid-algorithm2/home-post_decision_sweep_time" "/media/jtebert/data2/hybrid-algorithm2/lab-post_decision_sweep_time" -i post_data_dirs_split_#.txt 0 1 2 3`
    - For home computer: `python manage.py rename "/media/jtebert/data2/hybrid-algorithm2/lab-post_decision_sweep_time" "/media/jtebert/home-data/hybrid-algorithm2/home-post_decision_sweep_time" -i post_data_dirs_split_#.txt 8 9 10 11 12 13 14 15`
    - For woodlab computer: `python manage.py rename "/media/jtebert/data2/hybrid-algorithm2/lab-post_decision_sweep_time" "/media/woodlab/DATAPART1/jtebert/grid-decisions-data/hybrid-algorithm2/woodlab-post_decision_sweep_time" -i post_data_dirs_split_#.txt 4 5 6 7`
  - Set up tmux session: `tmux` (To detach: `Ctrl`+`b`, `d`. To reattach: `tmux attach`)
  - Run respective threads
    - For home computer: `python manage.py run "./bin/gridbots_decisions" 8 9 10 11 12 13 14 15 -i post_data_dirs_split_#.txt`
    - For woodlab computer: `python manage.py run "./bin/gridbots_decisions" 4 5 6 7 -i post_data_dirs_split_#.txt`
- To copy data afterwards:
  - From home computer: `cp -pr /media/jtebert/home-data/hybrid-algorithm2/home-post_decision_sweep home-post_decision_sweep_time /media/jtebert/portable-data/hybrid-algorithm2/post_decision_sweep_time`
  - From woodlab computer: `cp -pr /media/woodlab/DATAPART1/jtebert/grid-decisions-data/hybrid-algorithm2/woodlab-post_decision_sweep_time /media/woodlab/portable-data/hybrid-algorithm2/post_decision_sweep_time`