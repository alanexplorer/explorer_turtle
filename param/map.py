#!/usr/bin/env python

import yaml

with open("/home/alanpereira/catkin_ws/src/explorer_turtle/param/map.yaml", 'r') as stream:
    try:
        ARUCO = yaml.safe_load(stream)
    except yaml.YAMLError as exc:
        print(exc)

print(ARUCO[0][2])
