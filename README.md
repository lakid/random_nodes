# Random Nodes

This is a repository for random ROS nodes designed to do random things.

## message_sync
This simple node will sync image topics and ground_truth pose and save them in a csv file. The corresponding images are also saved in the running folder.

## image_to_laser

Subscribes to an image topic, detects edges, and publishes the edge points as a laser scan with range in `ranges[]` and bearing in `intensities[]`.
