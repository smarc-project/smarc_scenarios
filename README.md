# smarc_scenarios
Contains code for specific simulations and real world deployments

## Pipe following simulation scenario

### 1. Install simulation

First, check https://github.com/smarc-project/smarc_simulations for how to set up the simulation environment.

### 2. Start tmux simulation session

When that works, we can start the pipe following scenario. Before starting, make
sure to source your workspace in the `.bashrc`, similar to: `source /path/to/your/catkin_ws/devel/setup.bash`.
The simulation environment is started using a tmux script. For information on how to
launch it, check https://github.com/smarc-project/smarc_utils/blob/master/README.md#smarc_bringup .

Now that you know how to use and navigate tmux, let's start the session using:
```
rosrun smarc_bringup pipe_following.sh
```

### 3. Manually follow the pipe using the keyboard teleop

In the fourth tmux tab, you can start the keyboard teleop window. Try
to use this to steer the auv to follow the pipe. Instructions on
how to steer the auv can be found in https://github.com/smarc-project/smarc_utils#smarc_keyboard_teleop .

### 4. Check some useful topics

The AUV has a camera that publishes on `/small_smarc_auv/small_smarc_auv/camera/camera_image` and
laser scanners on `small_smarc_auv/sss_left` and `small_smarc_auv/sss_right`. All these topics
can be visualized in rviz if we have set `world` as the base frame. We can also add a robot model
with robot description `small_smarc_auv/robot_description` to see the robot in rviz. Add a `tf`
to rviz to see the relative position of all the frames.

In addition, the auv publishes several topics that might be of interest, you can look at the topics:
```
/small_smarc_auv/current_velocity
/small_smarc_auv/fins/0/output
/small_smarc_auv/fins/1/output
/small_smarc_auv/fins/2/output
/small_smarc_auv/fins/3/output
/small_smarc_auv/imu
/small_smarc_auv/is_submerged
/small_smarc_auv/thrusters/0/is_on
/small_smarc_auv/thrusters/0/thrust
```

### 5. Create a system for following the pipe automatically

This is something that we need to discuss!

These are just some random hints that came to mind.

#### OpenCV

You will need to convert the `sensor_msgs/Image` to `opencv` images
using `cv_bridge`, see http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython .

To find the pipeline, you might want to calculate some gradients
https://docs.opencv.org/trunk/d7/d4d/tutorial_py_thresholding.html , followed by thresholding
https://docs.opencv.org/2.4/doc/tutorials/imgproc/threshold/threshold.html
or just go for some thresholding based on the color directly.

Another idea might be to use the Hough transform to find the straight lines outlining the pipe:
https://docs.opencv.org/3.0-beta/doc/py_tutorials/py_imgproc/py_houghlines/py_houghlines.html

#### TF

You will want to read about the transform buffers in ROS: http://wiki.ros.org/tf/Tutorials .
The coordinate system in your case is defined with respect to the `world` frame, which is fixed
with respect to the simulated world. You can use the transform to the auv `/smarc_auv/base_link` frame.

#### Controllers

Some simple P controller, https://en.wikipedia.org/wiki/Proportional_control , maybe?
Or just something simpler. Do we need to add a depth sonar or a pressure sensor to be
able to properly know the height? Uncomment these lines to get a sonar:
https://github.com/smarc-project/smarc_simulations/blob/master/smarc_auvs/models/small_smarc_auv/urdf/small_smarc_auv_base.urdf.xacro#L227
