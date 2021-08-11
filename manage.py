"""
This is for setting up and running parameter sweep experiments.

It's based on grid-decisions/manage.py (which is, in turn, based on
kilosim-experiments/run_experiments.py, but a lot better).
"""

import os
import copy
import itertools
import argparse
import subprocess
import glob
import multiprocessing
import json
import time
import datetime
from typing import List, Optional

import yaml
import numpy as np


def get_dirs(dirs_filename):
    # Load the list of directories
    with open(dirs_filename, 'r') as f:
        file_contents = f.read()
    return file_contents.split('\n')


def product_dict(**kwargs):
    """
    Get the cross product of all of the input keys and values. This is used for
    generating the cartesian cross of all of the parameter options.
    """
    keys = kwargs.keys()
    vals = kwargs.values()
    for instance in itertools.product(*vals):
        yield dict(zip(keys, instance))


def dict_get_path(dict_in, use_path):
    # Get the value from the dictionary, where (eg)
    # use_path='this/path/deep' -> dict_in['this']['path']['deep']
    path_split = use_path.split('/')
    for p in path_split:
        dict_in = dict_in[p]
    return dict_in


def dict_del_path(dict_in, use_path):
    # Delete the value at the given path (as a key), leaving the rest intact
    path_split = use_path.split('/')
    # Loop over all the keys except last
    for p in path_split[:-1]:
        dict_in = dict_in[p]
    # Delete using last key in path
    del dict_in[path_split[-1]]


def format_value(val) -> str:
    # format a raw dictionary value (scalar) or sub-dictionary
    # into a string for path naming
    if isinstance(val, list):
        return '[' + '.'.join([str(v) for v in val]) + ']'
    if not isinstance(val, dict):
        return str(val)
    else:
        # It's a dictionary!
        # DEFAULT: {'descent': {'hc_complete_thresh': 8, 'hc_table_timeout': 100}}
        return '{' + ','.join([f'{k}:'+format_value(v) for k, v in val.items()]) + '}'


def gen_configs(src_yml: str, out_filename: str,
                allow_overwrite: bool = False, descriptive_folders: bool = False):
    """
    Generate all of the possible configuration combinations based on the cross
    product of all vary_params options in the given src_yml. A YAML config file
    will be generated for each in a subfolder specified by data_dir_base, and
    the locations of all generated configs is saved to out_filename.

    @note Any folders that already exist will be skipped (you will be informed
    if this happens) to avoid accidentally overwriting data.

    Data will be output in the current directory as ``data_dirs.txt``

    Parameters
    ----------
    src_yml : str
        Name/location of YAML file containing the meta-configuration. It must
        contain the keys 'data_dir_base' (where to put the generated JSON config
        files), 'vary_params' (all the parameters and their values that will be
        combined for the parameter sweep), and 'fixed_params' (parameter
        required by the simulation that are the same across all conditions)
    out_filename : str
        Name of the file to save the names of the locations of the generated configs.
    allow_overwrite : bool
        Whether to overwrite configs where the directory already exists (defaults to False).
        If True, these will also be included in the out_filename; otherwise, out_filename will only
        contain the names of the directories that were created
    descriptive_folders : bool
        Whether to use descriptive subfolders for the generated configs (defaults to False).
        If true, the subfolder names will be the concatenation of all of the varied parameter names
        and values.
        If false, the subfolder names will be numerically indexed.
    """
    with open(src_yml) as f:
        src_config = yaml.load(f, Loader=yaml.FullLoader)

    # This is a list of all of the NAMES of the variables in the YAML file that will be varied
    # All other variables are assumed to be fixed
    vary_params = src_config['vary_params']
    # The others are figured out dynamically, but putting them here helps for analysis
    vary_params_shallow = [p for p in vary_params if '/' not in p]
    vary_params_deep = [p for p in vary_params if '/' in p]
    data_dir_base = os.path.expanduser(src_config['data_dir_base'])
    vary_param_dict = {var: src_config[var] for var in vary_params_shallow}

    print('Data base directory:', data_dir_base)

    # Cross product of all varied parameters
    all_conditions = list(product_dict(**vary_param_dict))

    for sub_param_path in vary_params_deep:
        try:
            dict_get_path(src_config, sub_param_path)
        except KeyError:
            raise KeyError(f'Invalid vary_param "{sub_param_path}" (not found in {src_yml})')
        # path_split = sub_param_path.split('/')
        # base_param_name = path_split[0].rstrip('_params')
        # base_param_val = path_split[1]

    # Deal with additional cases where there's a config for specific sub-variants
    # (eg movement_type_params)
    for param_name in vary_param_dict:
        sub_param_name = param_name + '_params'
        if sub_param_name in src_config:
            # There are sub-parameters; look at values for this param_name within that
            for param_val in src_config[param_name]:  # loop over options for this parameter
                if param_val in src_config[sub_param_name]:
                    # check if there's extra configuration in this [*_params] group
                    # Remove this existing item from general configuration
                    specific_options = {
                        k: v for k, v in src_config[sub_param_name][param_val].items()
                        if isinstance(v, list)}
                    # single_options = {
                    #     k: v for k, v in src_config[sub_param_name][param_val].items()
                    #     if not isinstance(v, list)}
                    if len(specific_options) != 0:
                        sub_conditions = product_dict(**specific_options)
                        sub_conditions = list([{sub_param_name: {param_val: s_c}}
                                               for s_c in sub_conditions])
                        # - Get rid of these existing conditions from all_conditions
                        # - Add in ALL the subcondition options for each relevant_condition
                        # - Put the new relevant_conditions back into all_conditions
                        relevant_conditions = [c for c in all_conditions
                                               if c[param_name] == param_val]
                        all_conditions = [c for c in all_conditions
                                          if c[param_name] != param_val]
                        for condition in relevant_conditions:
                            all_conditions.extend([dict(condition, **s_c)
                                                   for s_c in sub_conditions])

    if descriptive_folders:
        data_dirs = [os.path.join(data_dir_base,
                                  '-'.join(f'{k}={format_value(v)}' for k, v in config.items()))
                     for config in all_conditions]
    else:
        # Number the directories
        num_dirs = len(all_conditions)
        z_len = len(str(num_dirs))
        data_dirs = [os.path.join(data_dir_base, f'params-{str(i).zfill(z_len)}')
                     for i in range(num_dirs)]

    print(f'Generating {len(all_conditions)} conditions...')

    new_data_dirs = []
    num_dirs = len(data_dirs)
    for ind, varied_config, data_dir in zip(range(num_dirs), all_conditions, data_dirs):
        if int(ind % int(num_dirs/80)) == 0:
            print('.', end="", flush=True)
        # Combine fixed and varied parameters
        # out_config = src_config.deepcopy()
        out_config = copy.deepcopy(src_config)
        # [out_config.pop(param) for param in remove_varied_sub_params]
        [dict_del_path(out_config, p) for p in vary_params_deep]
        out_config['log_dir'] = os.path.join(data_dir, '')  # Ensure trailing slash
        out_config.update(varied_config)
        # Create data directory
        if allow_overwrite or (not allow_overwrite and not os.path.exists(data_dir)):
            # If overwrites are allowed OR if the directory doesn't exist
            os.makedirs(data_dir, exist_ok=True)
            # print("CREATING\t", data_dir)
            new_data_dirs.append(data_dir)
        else:
            # print('SKIPPING\t', data_dir)
            pass
        # Dump to config.json in that location
        config_filename = os.path.join(data_dir, 'config.json')
        with open(config_filename, 'w') as config_file:
            json.dump(out_config, config_file)
        # with open('data.yml', 'w') as outfile:
        #     yaml.dump(data, outfile, default_flow_style=False)
    print()

    # Save the directory names to a file (ONLY DIRS THAT WERE CREATED)
    with open(out_filename, 'w') as f:
        f.write('\n'.join(new_data_dirs))
    print(f'Created and saved {len(new_data_dirs)} new configurations (saved in {out_filename}).')
    print(f'Skipped {len(all_conditions)-len(new_data_dirs)} existing configurations')


def split_data_dirs(filename, num_threads):
    """
    Split an existing text file of directories (each for one set of conditions)
    into multiple text files to run as different processes.

    Parameters
    ----------
    filename : str
        Source text file containing list of directories with JSON files for
        running experiments. This is the output of running this script with the
        "generate" command.
    num_threads : int
        Number of files to split the data file into. If this is being run on a
        single computer, this should be no more than the number of physical
        cores on the machine.
    """
    data_dirs = get_dirs(filename)
    out_dir = os.path.dirname(filename)

    filename_base = os.path.splitext(filename)[0]

    data_dir_groups = np.array_split(data_dirs, num_threads)

    # Each process then runs through its group sequentially
    for ind, group in enumerate(data_dir_groups):
        data_split_filename = os.path.join(out_dir, f'{filename_base}_split_{ind}.txt')
        print(ind, ":", data_split_filename)
        with open(data_split_filename, 'w') as f:
            f.write('\n'.join(group))


def run_groups(exec_filename: str, dirs_filename_glob: str,
               split_nums: Optional[List[int]] = None,):
    """Run all of the data_dirs splits in parallel. It will use as many threads as there are files.

    Parameters
    ----------
    exec_filename : str
        Name of the file to execute (will be passed to run_group_sequential)
    dirs_filename_glob : str
        Format of the data_dirs splits, eg "data_dirs_split_*.txt"
    """
    if split_nums is None:
        data_dirs_filenames = glob.glob(dirs_filename_glob)
    else:
        # Use the split numbers to get the filenames
        data_dirs_filenames = [dirs_filename_glob.replace('*', str(num)) for num in split_nums]
    print(data_dirs_filenames)
    input_pairs = [(exec_filename, f) for f in data_dirs_filenames]
    with multiprocessing.Pool(len(data_dirs_filenames)) as p:
        p.starmap(run_group_sequential, input_pairs)


def run_all_self_split(exec_filename: str, orig_data_dirs_filename: str, num_threads: int):
    """
    IGNORE ANY SPLITS and run all the data_dirs in parallel using multiprocessing.Pool with a given
    number of threads. This is more efficient because the pools will be re-used and there won't be
    waiting for slow (unlucky) groups. [<- I think that's actually not true.]

    Parameters
    ----------
    exec_filename : str
        Name of the file to execute as the main experiment file (which takes the config file as an
        argument).
    orig_data_dirs_filename : str
        Name of the text file specifying the locations of YAML config files for
        which to run experiments.
    num_threads : int
        Number of threads/pools to split up the grouping into
    """
    data_dirs = get_dirs(orig_data_dirs_filename)
    input_pairs = [(exec_filename, d) for d in data_dirs]
    with multiprocessing.Pool(num_threads) as p:
        p.starmap(run_single, input_pairs)


def run_single(exec_filename, data_dir):
    # Run a single instance
    # This just makes things easier to read
    config_file = os.path.join(data_dir, 'config.json')
    # subprocess.call(['python', exec_filename, config_file])
    subprocess.call([exec_filename, config_file])


def run_group_sequential(exec_filename: str, dirs_filename: str):
    """
    Run all of the experiments specified in the given text file. Trials will be
    run sequentially.

    Parameters
    ----------
    exec_filename : str
        Name of the file to execute as the main experiment file (which takes the config file as an
        argument).
    dirs_filename : str
        Name of the text file specifying the locations of YAML config files for
        which to run experiments.
    """
    print(dirs_filename)
    data_dirs = get_dirs(dirs_filename)
    for data_dir in data_dirs:
        run_single(exec_filename, data_dir)
        # config_file = os.path.join(data_dir, 'config.yml')
        # subprocess.call(['python', exec_filename, config_file])


def check_progress(dir: str, num_splits: int, num_cores: int):
    """Check how much of the parameter sweep has been run so far.

    This uses checking the number of files in the directory to check whether a
    condition has been started yet.

    Parameters
    ----------
    dir : str
        Location to check for subfolders and data files
    num_splits : int
        Number of sections/chunks the runs are split into
    num_cores : int
        Number of cores used to run the simulations on this computer
    """
    # There's one file (config.json) in each subfolder before anything is run
    # subsolders = [d for d in os.listdir(dir) if os.path.isdir(os.path.join(dir, d))]
    subsolders = [d for d in os.listdir(dir) if os.path.isdir(os.path.join(dir, d))]
    total_dirs = len(subsolders)
    started_dirs = 0
    completed_dirs = 0
    last_data_time = np.nan
    first_data_time = np.nan
    for subdir in subsolders:
        # Get the time that data.h5 was last modified
        data_file = os.path.join(dir, subdir, 'data.h5')
        if os.path.exists(data_file):
            data_mod_time = os.path.getmtime(data_file)
            data_create_time = os.path.getctime(data_file)
            last_data_time = np.nanmax([data_mod_time, last_data_time])
            first_data_time = np.nanmin([data_create_time, first_data_time])
            started_dirs += 1
            # Consider a file completed if it hasn't been modifed in the last 5 minutes
            if (time.time() - data_mod_time) >= 300:
                completed_dirs += 1

    percent_started = 100*started_dirs/total_dirs
    print(f'{started_dirs} of {total_dirs} subdirectories started ({percent_started:.2f}%)')
    completed_count = max(0, started_dirs-num_cores, completed_dirs)
    percent_done = 100 * completed_count/total_dirs
    print(f'{completed_count} of {total_dirs} subdirectories done ({percent_done:.2f}%)')

    # Roughly estimate time remaining
    time_elapsed = last_data_time - first_data_time
    time_elapsed_dt = datetime.timedelta(seconds=time_elapsed)
    # time_remaining = time_elapsed / (started_dirs / num_splits) * (num_cores - started_dirs)
    # Time per file (don't include currently edited)
    # all times are in seconds (since epoch)
    # completed_dirs = max(started_dirs - num_cores, 0)  # can't be negative
    if completed_count != 0:
        time_per_file = time_elapsed / completed_count
        total_time = time_per_file * total_dirs
        time_remaining = total_time - time_elapsed
        completion_time = first_data_time + total_time
        print(f'Time elapsed: {time_elapsed_dt}')
        print(f'Time remaining: {datetime.timedelta(seconds=time_remaining)}')
        print(f'Expected completion: {time.ctime(completion_time)}')


if __name__ == "__main__":

    DATA_DIRS_FILE_BASE = 'data_dirs'
    DATA_DIRS_FILE_EXT = '.txt'
    DATA_DIRS_FILENAME = DATA_DIRS_FILE_BASE + DATA_DIRS_FILE_EXT
    DATA_DIRS_SPLIT_FILEFORMAT = DATA_DIRS_FILE_BASE + '_split_#' + DATA_DIRS_FILE_EXT

    parser = argparse.ArgumentParser()

    subparser = parser.add_subparsers(
        title='command',
        help="Choose whether to generate a set of configurations for a " +
        "parameter sweep, split such a list of configurations, or run all the" +
        " configurations in a given file."
    )

    progress_parser = subparser.add_parser("progress")
    progress_parser.set_defaults(cmd='progress')
    progress_parser.add_argument(
        "dir",
        type=str,
        help="Directory to look for progress in"
    )
    progress_parser.add_argument(
        "num_splits",
        type=int,
        help="Number of sections/chunks to split the parameter sweep into"
    )
    progress_parser.add_argument(
        "num_cores",
        type=int,
        help="Number of cores running on this machine for the sweep"
    )

    gen_parser = subparser.add_parser("generate")
    gen_parser.set_defaults(cmd="generate")
    gen_parser.add_argument(
        "src",
        type=str,
        help="YAML source file for generating config files"
    )
    gen_parser.add_argument(
        "--allow_overwrite",
        default=False,
        action='store_true',
        help="Whether to allow recreating directories (will be included in data_dirs.txt)"
    )
    gen_parser.add_argument(
        "-o", "--outfile",
        nargs='?',
        type=str,
        default=DATA_DIRS_FILENAME,
        help="Name of text file to output data directories"
    )
    gen_parser.add_argument(
        "--descriptive_folders",
        default=True,
        action='store_true',
        help='Whether to use descriptive names for the folders' +
             ' (ie, including varied parameter names/values)'
    )

    split_parser = subparser.add_parser("split")
    split_parser.set_defaults(cmd='split')
    split_parser.add_argument(
        "num_splits",
        type=int,
        help="Number of files you want to split the input into")
    split_parser.add_argument(
        "-i", "--infile",
        nargs='?',
        type=str,
        default=DATA_DIRS_FILENAME,
        help="Name of input file you want to split")

    run_parser = subparser.add_parser("run")
    run_parser.set_defaults(cmd='run')
    run_parser.add_argument(
        'exec_file',
        help='Path to the executable main experiment file to run'
    )
    run_parser.add_argument(
        "split_nums",
        nargs="+",
        help="Run all the trials in infile with #=split_num. Use 'all' to run all split files")
    run_parser.add_argument(
        '-i', '--infile',
        nargs='?',
        type=str,
        default=DATA_DIRS_SPLIT_FILEFORMAT,
        help='Format of input files, with # as number wildcard. eg: data_dirs_split_#.txt (default)'
    )

    split_run_parser = subparser.add_parser('split_run')
    split_run_parser.set_defaults(cmd='split_run')
    split_run_parser.add_argument(
        'exec_file',
        help='Path to the executable main experiment file to run'
    )
    split_run_parser.add_argument(
        'num_threads',
        type=int,
        help='Number of threads (pools) to split data_dirs.txt into to run.'
    )
    split_run_parser.add_argument(
        "-i", "--infile",
        nargs='?',
        type=str,
        default=DATA_DIRS_FILENAME,
        help="Name of input file you want to split")

    args = parser.parse_args()

    if ("cmd" not in args):
        print("You must specify a command option to run: generate, split, or run.")
    else:
        if args.cmd == 'generate':
            print('Generating configurations')
            print('Source:', args.src)
            print('Output:', args.outfile)
            gen_configs(args.src, args.outfile,
                        allow_overwrite=args.allow_overwrite,
                        descriptive_folders=args.descriptive_folders)

        elif args.cmd == 'split':
            print(f'Splitting {args.infile} into [{args.num_splits}] parts')
            split_data_dirs(args.infile, args.num_splits)

        elif args.cmd == 'run':
            if args.split_nums[0] == 'all':
                split_nums = 'all'
            else:
                split_nums = [int(n) for n in args.split_nums]

            if '#' not in args.infile:
                print('ERROR: Split filename must contain "#" to indicate the numbers.')
            filename_glob = args.infile.replace('#', '*')
            if split_nums == 'all':
                print("Running all", filename_glob)
                run_groups(args.exec_file, filename_glob)
            else:
                print(f"Running {len(split_nums)} groups: {split_nums}")
                run_groups(args.exec_file, filename_glob, split_nums)

        elif args.cmd == 'split_run':
            print(f'Splitting "{args.infile}" into {args.num_threads} and running all')
            run_all_self_split(args.exec_file, args.infile, args.num_threads)

        elif args.cmd == 'progress':
            check_progress(args.dir, args.num_splits, args.num_cores)

        else:
            print("Invalid command. Choose 'generate', 'split', 'run', or 'split_run'")
