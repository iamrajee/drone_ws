#!/bin/bash
rosdep install --from-paths src --ignore-src --rosdistro noetic -y -r
rosdep update
