# The Pupil Labs wrapper for ROS2

## Dependencies
You will want to make sure that you have the most recent version of ros2 but any version should work. Additionally you will want make sure you install the pupil_labs_realtimeapi.

### Follow the steps on these pages 
[ROS2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html)

[Pupil Labs API](https://docs.pupil-labs.com/neon/real-time-api/tutorials/)
## The Wrapper
In order to run this wrapper you must clone the directory into your ros2 workspace and then build the workspace. 

```git clone https://github.com/UW-CTRL/pupil_labs_ros2_wrapper.git```

```colcon build```

## The Setup 
Now in order to get the wrapper up and running, you must create a python virtual enviornment and then enter it:

```python -m venv /path/to/virtual/environment```

```source path/to/virtual/environment/bin/activate```

then you must export the path to your pupil labs api:

```export PYTHONPATH=/home/kysh/venv/pupil_labs/lib/python3.12/site-packages:$PYTHONPATH```

## Done 

Now you can build one more time and then run the wrapper and see the published messages for the eye_img, scene_img, and gaze:

```colcon build```

```ros2 run pupil_labs_wrapper wrapper.py```


