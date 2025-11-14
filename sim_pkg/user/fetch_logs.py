#!/usr/bin/env python3.8
import os
import argparse
import asyncio
from argparse import Namespace

from cctl import cli
from cctl.conf import Configuration

async def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--ids", nargs="+", required=True,
        help="Robot *hardware* IDs to fetch logs from, e.g. 10 11 12"
    )
    parser.add_argument(
        "--out", default="./logs",
        help="Output directory for fetched logs"
    )
    args = parser.parse_args()

    out_dir = os.path.abspath(args.out)
    os.makedirs(out_dir, exist_ok=True)

    robot_args = Namespace()
    robot_args.id = args.ids      # list of strings like ['10', '11']
    robot_args.output_dir = out_dir

    # This is the same handler used in the automation script (commented there)
    await cli.commands.fetch_output_handler(robot_args, Configuration())

if __name__ == "__main__":
    asyncio.run(main())
