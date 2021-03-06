#!/usr/local/bin/python3
# -*- coding: UTF-8 -*-

from argparse import ArgumentParser
import sys
import subprocess
import logging


logging.basicConfig(format='%(asctime)s [%(levelname)s]: %(message)s', level=logging.DEBUG)

def main(exe: str, config: str, ref: str):
    res = subprocess.run(f'"{exe}" "{config}" > tmp.log', timeout=2, shell=True, capture_output=True)
    res.check_returncode()

    with open('tmp.log', 'r') as tmp_log, open(ref, 'r') as ref_log:
        read_lines = 0
        same_lines = 0
        for line1, line2 in zip(ref_log, tmp_log):
            read_lines += 1
            if not line1.startswith('[DEBUG]'):
                continue
            if line1.strip() != line2.strip():
                logging.warning(f'Log file does not match at line {read_lines}!')
                logging.warning(f'ref: {line1}')
                logging.warning(f'cur: {line2}')
                exit(-1)
            else:
                same_lines += 1
        if same_lines == 0:
            logging.warning('No same lines!')
            exit(-1)

if __name__ == '__main__':
    parser = ArgumentParser()
    parser.add_argument('--exe', required=True, help='the path to executable', type=str)
    parser.add_argument('--config', required=True, help='the path to configuration file', type=str)
    parser.add_argument('--ref', required=True, help='the reference file', type=str)
    args = parser.parse_args(sys.argv[1:])

    try:
        main(args.exe, args.config, args.ref)
    except subprocess.CalledProcessError as e:
        logging.error(f'Exit with non-zero code: {e.returncode}')
        logging.error(e.stderr)
        exit(-1)
