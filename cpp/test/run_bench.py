#!/usr/local/bin/python3
# -*- coding: UTF-8 -*-

from argparse import ArgumentParser
import json
from pathlib import Path
import re
import sys
import subprocess
import logging

logging.basicConfig(format='%(asctime)s [%(levelname)s]: %(message)s', level=logging.INFO)


def main(exe: Path, config: Path, tmp_dir: Path):
    tmp_json = tmp_dir.joinpath('bench.json')
    with open(config, 'r') as cfp, open(tmp_json, 'w') as jfp:
        config_obj = json.load(cfp)
        config_obj['data_folder'] = str(tmp_dir)
        json.dump(config_obj, jfp)

    run_cnt = 6
    cmd = f'"{str(exe)}" -f "{str(tmp_json)}" -n {run_cnt}'
    res = subprocess.run(cmd, timeout=60, shell=True, capture_output=True)
    res.check_returncode()

    ray_cnt = []
    tracing_time = []
    collecting_time = []
    elapsed_time = []
    curr_tracing_time = 0
    curr_collecting_time = 0
    for line in res.stdout.splitlines():
        logging.debug(line)

        # Tracing time
        m = re.search(b'\[INFO\]\([a-z0-9]*\) Ray tracing: (.*)ms', line)
        if m:
            curr_tracing_time += float(m.group(1))

        # Collecting time
        m = re.search(b'\[INFO\]\([a-z0-9]*\) Collecting rays: (.*)ms', line)
        if m:
            curr_collecting_time += float(m.group(1))

        # Ray numbers
        m = re.search(b'\[INFO\]\([a-z0-9]*\) === Total\s+([0-9.]*)M rays', line)
        if m:
            ray_cnt.append(float(m.group(1)))

        # Total time
        m = re.search(b'\[INFO\]\([a-z0-9]*\) === Spent\s+([0-9.]*) sec', line)
        if m:
            elapsed_time.append(float(m.group(1)))
            tracing_time.append(curr_tracing_time)
            collecting_time.append(curr_collecting_time)
            curr_tracing_time = 0
            curr_collecting_time = 0

    logging.info(f'Tracing time: {sum(tracing_time[1:]) / (run_cnt - 1):.2f}ms')
    logging.info(f'Collecting time: {sum(collecting_time[1:]) / (run_cnt - 1):.2f}ms')
    logging.info(f'Average time: {(elapsed_time[-1] - elapsed_time[0]) / (ray_cnt[-1] - ray_cnt[0]):.2f} sec/M rays')


if __name__ == '__main__':
    parser = ArgumentParser()
    parser.add_argument('--exe', required=True, help='the path to executable', type=str)
    parser.add_argument('--config', required=True, help='the path to configuration file', type=str)
    parser.add_argument('--tmp_dir', required=True, help='temperary directory for data', type=str)
    parser.add_argument('-v', required=False, help='make output verbose', type=str)
    args = parser.parse_args(sys.argv[1:])

    if args.v:
        logging.basicConfig(format='%(asctime)s [%(levelname)s]: %(message)s', level=logging.DEBUG)

    try:
        main(Path(args.exe), Path(args.config), Path(args.tmp_dir))
    except subprocess.CalledProcessError as e:
        logging.error(f'Exit with non-zero code: {e.returncode}')
        logging.error(e.stderr)
        exit(-1)
