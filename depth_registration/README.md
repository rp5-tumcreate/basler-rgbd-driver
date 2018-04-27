
## Depth Registration

The depth registration nodelet takes in Depth Image, RGB Camera Matrix and Depth Camera Matrix and a RGB to TOF frame through TF to generate a registered depth map on the GPU. It also outputs the new registered depth camera matrix depending on the resolution you chose. This nodelet is also capable of doing calibration between RGB and TOF camera. This nodelet requires that your cameras confirm to certain topic formats. For example:

```
if(republishedRgb_)
subCameraImageTo_.subscribe(nh, cameraNameTo_ + "/image_color_rect/republished", 1);
else
subCameraImageTo_.subscribe(nh, cameraNameTo_ + "/image_color_rect", 1);
subCameraImageFrom_.subscribe(nh, cameraNameFrom_ + "/depth_rect", 1);
subCameraInfoFrom_.subscribe(nh, cameraNameFrom_ + "/camera_info", 1);
subCameraInfoTo_.subscribe(nh, cameraNameTo_ + "/camera_info", 1);
```

### Usage

```
<!-- Registration -->
<node pkg="nodelet" type="nodelet" name="$(arg base_tof_name)_depth_registration" machine="$(arg machine)"
args="load depth_registration/depth_registration $(arg nodelet_manager)" respawn="$(arg respawn)" if="$(arg transmitter)">
<param name="queue_size" type="int" value="$(arg queue_size)"/>
<param name="calibration_mode" type="bool" value="$(arg calibrate)"/>
<param name="camera_name_from" type="string" value="$(arg base_tof_name)"/>
<param name="camera_serial_from" type="string" value="22100546"/>
<param name="camera_name_to" type="string" value="$(arg base_rgb_name)"/>
<param name="camera_serial_to" type="string" value="22099567"/>
<rosparam param="registered_resolution">[640, 480]</rosparam>
</node>
```

You can use turn on and off the `calibration_mode` flag, more information on that in the next section. It should be off by default. `camera_name_from` and `camera_name_to` represent the name of the cameras you want to transform the depth from and to. Serial numbers will also need to be passed in for the purpose of saving calibration data. `registered_resolution` allows you to specify the resolution of your new registered camera.  The node then outputs new topics as follows:

```
pubDepthRegistered_ = nh.advertise<sensor_msgs::Image>(cameraNameTo_ + "/registered/depth_rect", 1, connect_cb, connect_cb);
pubRGBRegistered_ = nh.advertise<sensor_msgs::Image>(cameraNameTo_ + "/registered/image_color_rect", 1, connect_cb, connect_cb);
pubCameraInfo_ = nh.advertise<sensor_msgs::CameraInfo>(cameraNameTo_ + "/registered/camera_info", 1, connect_cb, connect_cb);
```

### Calibration

If the `calibration_mode` flag is set to true, it allows the node to perform extrinsic calibration between the RGB and Depth cameras. This requires the following topics:

```
subCameraImageFrom_.subscribe(nh, cameraNameFrom_ + "/intensity_rect", 1);
subCameraImageTo_.subscribe(nh, cameraNameTo_ + "/image_color_rect", 1);
subCameraInfoFrom_.subscribe(nh, cameraNameFrom_ + "/camera_info", 1);
subCameraInfoTo_.subscribe(nh, cameraNameTo_ + "/camera_info", 1);
```

You can then start and observer the calibration process below as an example:

```
# Run this service call (It is blocking)
rosservice call /TOF0_depth_registration/extrinsics_calibrate_start "board_size: '9x6'
square_size: 0.117"
# Open rqt and look for the /TOF0_depth_registration/calibrated_preview and /TOF0_depth_registration/calibration_preview
# Point camera at different board poses, ensure the board is seen in both cameras
rosservice call /TOF0_depth_registration/extrinsics_calibrate_signal "save: false"
# Call this several times for different poses. When you want to stop
rosservice call /TOF0_depth_registration/extrinsics_calibrate_signal "save: true"
# Observer the reprojection error. Ensure it is less than 3 pixels
```

### Algorithm

More information on how the calibration and registration algorithm occurs click [here](https://github.com/rp5-tumcreate/ac2-ros-sensor-pkgs/blob/nuc-linux-devel/docs/tex/camera/Camera.pdf).

