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

    start_line = 16
    with open('tmp.log', 'r') as tmp_log, open(ref, 'r') as ref_log:
        read_lines = 0
        for line1, line2 in zip(ref_log, tmp_log):
            read_lines += 1
            if read_lines < start_line:
                continue
            if line1.startswith('Total:'):
                continue
            if line1.strip() != line2.strip():
                logging.warn(f'Log file does not match at line {read_lines}!')
                logging.warn(f'ref: {line1}')
                logging.warn(f'cur: {line2}')
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
