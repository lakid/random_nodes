# Random Nodes
**Author: Lakshitha Dantanarayana**
**License: BSD 2-Clause**

This is a repository for random ROS nodes designed to do random things. Usually hacked together in the shortest possible time.

## message_sync
This simple node will sync image topics and ground_truth pose and save them in a csv file. The corresponding images are also saved in the running folder.

## image_to_laser

Subscribes to an image topic, detects edges, and publishes the edge points as a laser scan with range in `ranges[]` and bearing in `intensities[]`.

## read_bag

Loads a rosbag file into memory synchronising given topics. A ros service interface is then registered and the messages can be queried by giving the sequence number.

## interactive_map_maker

Simple service client to the above node.
