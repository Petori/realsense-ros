
## Usage Instructions
- `roslaunch realsense2_camera rs_camera.launch`
- `rosrun rqt_image_view rqt_image_view`


## Official
This will stream all camera sensors and publish on the appropriate ROS topics.

Other stream resolutions and frame rates can optionally be provided as parameters to the 'rs_camera.launch' file.

### Published Topics
The published topics differ according to the device and parameters.
After running the above command with D435i attached, the following list of topics will be available (This is a partial list. For full one type `rostopic list`):
- /camera/color/camera_info
- /camera/color/image_raw
- /camera/depth/camera_info
- /camera/depth/image_rect_raw
- /camera/extrinsics/depth_to_color
- /camera/extrinsics/depth_to_infra1
- /camera/extrinsics/depth_to_infra2
- /camera/infra1/camera_info
- /camera/infra1/image_rect_raw
- /camera/infra2/camera_info
- /camera/infra2/image_rect_raw
- /camera/gyro/imu_info
- /camera/gyro/sample
- /camera/accel/imu_info
- /camera/accel/sample
- /diagnostics

The "/camera" prefix is the default and can be changed. Check the **rs_multiple_devices.launch** file for an example.
If using D435 or D415, the gyro and accel topics wont be available. Likewise, other topics will be available when using T265 (see below).

### Launch parameters
The following parameters are available by the wrapper:
- **serial_no**: will attach to the device with the given serial number (*serial_no*) number. Default, attach to available RealSense device in random.
- **usb_port_id**: will attach to the device with the given USB port (*usb_port_id*). i.e 4-1, 4-2 etc. Default, ignore USB port when choosing a device.
- **device_type**: will attach to a device whose name includes the given *device_type* regular expression pattern. Default, ignore device type. For example, device_type:=d435 will match d435 and d435i. device_type=d435(?!i) will match d435 but not d435i.

- **rosbag_filename**: Will publish topics from rosbag file.
- **initial_reset**: On occasions the device was not closed properly and due to firmware issues needs to reset. If set to true, the device will reset prior to usage.
- **align_depth**: If set to true, will publish additional topics with the all the images aligned to the depth image.</br>
The topics are of the form: ```/camera/aligned_depth_to_color/image_raw``` etc.
- **filters**: any of the following options, separated by commas:</br>
 - ```colorizer```: will color the depth image. On the depth topic an RGB image will be published, instead of the 16bit depth values .
 - ```pointcloud```: will add a pointcloud topic `/camera/depth/color/points`. The texture of the pointcloud can be modified in rqt_reconfigure (see below) or using the parameters: `pointcloud_texture_stream` and `pointcloud_texture_index`. Run rqt_reconfigure to see available values for these parameters.</br>
 The depth FOV and the texture FOV are not similar. By default, pointcloud is limited to the section of depth containing the texture. You can have a full depth to pointcloud, coloring the regions beyond the texture with zeros, by setting `allow_no_texture_points` to true.

 - The following filters have detailed descriptions in : https://github.com/IntelRealSense/librealsense/blob/master/doc/post-processing-filters.md
   - ```disparity``` - convert depth to disparity before applying other filters and back.
   - ```spatial``` - filter the depth image spatially.
   - ```temporal``` - filter the depth image temporally.
   - ```hole_filling``` - apply hole-filling filter.
   - ```decimation``` - reduces depth scene complexity.
- **enable_sync**: gathers closest frames of different sensors, infra red, color and depth, to be sent with the same timetag. This happens automatically when such filters as pointcloud are enabled.
- ***<stream_type>*_width**, ***<stream_type>*_height**, ***<stream_type>*_fps**: <stream_type> can be any of *infra, color, fisheye, depth, gyro, accel, pose*. Sets the required format of the device. If the specified combination of parameters is not available by the device, the stream will not be published. Setting a value to 0, will choose the first format in the inner list. (i.e. consistent between runs but not defined). Note: for gyro accel and pose, only _fps option is meaningful.
- **enable_*<stream_name>***: Choose whether to enable a specified stream or not. Default is true. <stream_name> can be any of *infra1, infra2, color, depth, fisheye, fisheye1, fisheye2, gyro, accel, pose*.
- **tf_prefix**: By default all frame's ids have the same prefix - `camera_`. This allows changing it per camera.
- **base_frame_id**: defines the frame_id all static transformations refers to.
- **odom_frame_id**: defines the origin coordinate system in ROS convention (X-Forward, Y-Left, Z-Up). pose topic defines the pose relative to that system.
- **All the rest of the frame_ids can be found in the template launch file: [nodelet.launch.xml](./realsense2_camera/launch/includes/nodelet.launch.xml)**
- **unite_imu_method**: The D435i and T265 cameras have built in IMU components which produce 2 unrelated streams: *gyro* - which shows angular velocity and *accel* which shows linear acceleration. Each with it's own frequency. By default, 2 corresponding topics are available, each with only the relevant fields of the message sensor_msgs::Imu are filled out.
Setting *unite_imu_method* creates a new topic, *imu*, that replaces the default *gyro* and *accel* topics. The *imu* topic is published at the rate of the gyro. All the fields of the Imu message under the *imu* topic are filled out.
 - **linear_interpolation**: Every gyro message is attached by the an accel message interpolated to the gyro's timestamp.
 - **copy**: Every gyro message is attached by the last accel message.
- **clip_distance**: remove from the depth image all values above a given value (meters). Disable by giving negative value (default)
- **linear_accel_cov**, **angular_velocity_cov**: sets the variance given to the Imu readings. For the T265, these values are being modified by the inner confidence value.
- **hold_back_imu_for_frames**: Images processing takes time. Therefor there is a time gap between the moment the image arrives at the wrapper and the moment the image is published to the ROS environment. During this time, Imu messages keep on arriving and a situation is created where an image with earlier timestamp is published after Imu message with later timestamp. If that is a problem, setting *hold_back_imu_for_frames* to *true* will hold the Imu messages back while processing the images and then publish them all in a burst, thus keeping the order of publication as the order of arrival. Note that in either case, the timestamp in each message's header reflects the time of it's origin.
- **topic_odom_in**: For T265, add wheel odometry information through this topic. The code refers only to the *twist.linear* field in the message.
- **calib_odom_file**: For the T265 to include odometry input, it must be given a [configuration file](https://github.com/IntelRealSense/librealsense/blob/master/unit-tests/resources/calibration_odometry.json). Explanations can be found [here](https://github.com/IntelRealSense/librealsense/pull/3462). The calibration is done in ROS coordinates system.
- **publish_tf**: boolean, publish or not TF at all. Defaults to True.
- **tf_publish_rate**: double, positive values mean dynamic transform publication with specified rate, all other values mean static transform publication. Defaults to 0
- **publish_odom_tf**: If True (default) publish TF from odom_frame to pose_frame.


### RGBD Point Cloud
Here is an example of how to start the camera node and make it publish the RGBD point cloud using aligned depth topic.
```bash
roslaunch realsense2_camera rs_camera.launch filters:=pointcloud
```
Then open rviz to watch the pointcloud:
<p align="center"><img src="https://user-images.githubusercontent.com/17433152/35396613-ddcb1d6c-01f5-11e8-8887-4debf178d0cc.gif" /></p>

### Aligned Depth Frames
Here is an example of how to start the camera node and make it publish the aligned depth stream to other available streams such as color or infra-red.
```bash
roslaunch realsense2_camera rs_camera.launch align_depth:=true
```
<p align="center"><img width=50% src="https://user-images.githubusercontent.com/17433152/35343104-6eede0f0-0132-11e8-8866-e6c7524dd079.png" /></p>

### Set Camera Controls Using Dynamic Reconfigure Params
The following command allow to change camera control values using [http://wiki.ros.org/rqt_reconfigure].
```bash
rosrun rqt_reconfigure rqt_reconfigure
```
<p align="center"><img src="https://user-images.githubusercontent.com/40540281/55330573-065d8600-549a-11e9-996a-5d193cbd9a93.PNG" /></p>

### realsense2_description package:
For viewing included models, a separate package is included. For example:
```bash
roslaunch realsense2_description view_d415_model.launch
```

### Unit tests:
Unit-tests are based on bag files saved on S3 server. These can be downloaded using the following commands:
```bash
cd catkin_ws
wget "http://realsense-hw-public.s3.amazonaws.com/rs-tests/TestData/outdoors.bag" -P "records/"
wget "http://realsense-hw-public.s3-eu-west-1.amazonaws.com/rs-tests/D435i_Depth_and_IMU_Stands_still.bag" -P "records/"
```
Then, unit-tests can be run using the following command:
```bash
python src/realsense/realsense2_camera/scripts/rs2_test.py --all
```

## Packages using RealSense ROS Camera
| Title | Links |
| ----- | ----- |
| ROS Object Analytics | [github](https://github.com/intel/ros_object_analytics) / [ROS Wiki](http://wiki.ros.org/IntelROSProject)

## Known Issues
* This ROS node does not currently support [ROS Lunar Loggerhead](http://wiki.ros.org/lunar).
* This ROS node does not currently work with [ROS 2](https://github.com/ros2/ros2/wiki).
* This ROS node currently does not support running multiple T265 cameras at once. This will be addressed in a future update.

## License
Copyright 2018 Intel Corporation

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this project except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

**Other names and brands may be claimed as the property of others*
