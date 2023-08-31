#!/usr/bin/env python3
#
# example command
# rosrun sciurus_examples sciurus_post_process.py --bag_path /home/username/.ros/filename.bag --out_folder /home/turtle/.ros/test_results/artefacts_postprocess/metrics.json

import re
import rosbag
import argparse
import sys
import json


parser = argparse.ArgumentParser(description="sciurus post processing")
parser.add_argument("--bag_path", help="path to input ROS bag")
parser.add_argument("--out_folder", help="folder path for outputs (e.g .html files)")
parser.add_argument(
    "--skip_figures",
    default=False,
    action="store_true",
    help="flag: do not generate figure outputs",
)
parser.add_argument(
    "--skip_metrics",
    default=False,
    action="store_true",
    help="flag: do not dump metrics in json file",
)
args = vars(parser.parse_args())

# choose topics to extract from rosbag
takt_time_topic = "/artefacts/takt_time"

# preliminary check: make sure bag indexes are in increasing time order
with rosbag.Bag(args["bag_path"], "r") as bag:
    timelist = [msg.timestamp.to_sec() for msg in bag.read_messages()]
assert timelist == sorted(timelist)

# extract messages from rosbag
takt_time_messages = []
pattern = r"data:\s*(\d+\.\d+)"
with rosbag.Bag(args["bag_path"], "r") as bag:
    # takt_time_messages = str([_ for _ in bag.read_messages(topics=takt_time_topic)])
    full_message = str([_ for _ in bag.read_messages(topics=takt_time_topic)])
    match = re.search(pattern, full_message)
    takt_time_data_str = match.group(1)
    takt_time_data_float = float(takt_time_data_str)
    takt_time_messages.append(takt_time_data_float)

# export metrics as a .json
if not args["skip_metrics"]:
    metrics = {
        "takt_time": takt_time_messages[-1],
    }
    with open(args["out_folder"] + "/metrics.json", "w") as f:
        json.dump(metrics, f)
