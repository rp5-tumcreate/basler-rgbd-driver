## Basler Bridge

Basler Bridge contains drivers for extracting RGB and Depth Data from the combined RGB/Camera System. This driver can interface with the `od_compressed_image_transport`, `depth_registration` and `cloud_generator` nodes in order to complete the package. It also uses the `params_lib` library for useful library functions.

### Hardware Setup (New Camera)

##### Physical Setup

![](https://user-images.githubusercontent.com/3193196/32479656-dc1c8ee2-c3c5-11e7-9c2c-6457e4043802.png) 

You first need to physically purchase and setup the TOF and RGB cameras. Consult the mechanical people to get the mounting bracket and holding plate. Ensure that these are securely attached.

##### Power System

You will need a 12 pin and 6 pin `Hirose 
micro receptacle and plug` for the TOF and RGB respectively. These can be purchased directly from Basler or from Element14. The connection is as follows below.

![ ](https://user-images.githubusercontent.com/3193196/32479905-003e39d2-c3c7-11e7-8201-37d4ac853e0c.png  "TOF")

![ ](https://user-images.githubusercontent.com/3193196/32479972-60103fea-c3c7-11e7-9009-33af209ec4e7.png  "RGB")

You can use a shared `24V DC Power at up to 2Amps`. The power supply provided by Basler for the TOF can be used to support both cameras if you want to.

##### Network System

Ensure that the two shielded RJ45 cables are used and are connected to a `Gigabit Switch with Jumbo Frames support`. Most gigabit switches should support this. Then connect this switch to your computer.

##### Other

Before starting software setup, open the exposure of the physical RGB camera to maximum. Now mount the camera onto one of the tripods.

### Software Setup (New Camera)

##### Low level tuning

Ensure that the Basler drivers are installed and the cameras are turned on. If your packages compiled, the driver should be setup. Go to terminal and type `/opt/BaslerToF/bin/PylonViewerApp`. Double click the camera and ensure that you have a Depth Feed. 

![](https://user-images.githubusercontent.com/3193196/32480261-b9fb607e-c3c8-11e7-810a-8f80585550ba.png) 

Go to terminal and type `/opt/pylon5/bin/PylonViewerApp`. Double click the camera and ensure that you have a feed. Now tune your physical camera focus until objects within `1m to 6m` are clear. Use the checkboard for this. If the image is too dark, ensure that your camera physical exposure has been increased to maximum. If it is still too dark, click the `Automatic Image adjustment` button. 

![](https://user-images.githubusercontent.com/3193196/32480383-8a0ad218-c3c9-11e7-80b7-0f73fef57129.png) 

Try to make both cameras run together and see if there are any issues

##### Intrinsic Calibration

First, take note of what both camera serial numbers are. Then, go to the `basler_bridge/data` folder and make a copy of `22100546` (TOF) and `22099567` (RGB). Then rename these to your new TOF and RGB serial numbers. Start the driver in a terminal with the following command:

```
roslaunch basler_bridge basler_bridge.launch base_tof_name:=TOF0 base_tof_serial:=22218136 base_rgb_name:=RGB0 base_rgb_serial:=22410499 receiver:=true transmitter:=true
```

Place one of the calibration boards on the floor, and start the calibration process for instrinsics for both cameras. Ensure that a computer with `OpenCV 3.0 and above` is used. If not it won't work. Then, run the following commands:

```
# http://wiki.ros.org/camera_calibration
rosrun camera_calibration cameracalibrator.py --size 9x6 --square 0.117 image:=/TOF0/intensity   camera:=/TOF0
# Calibrate, then click commit
rosrun camera_calibration cameracalibrator.py --size 9x6 --square 0.117 image:=/RGB0/image_color camera:=/RGB0
# Calibrate, then click commit
```

##### TOF to RGB Extrinsic Calibration

Close the driver, and re-run the following command:

```
roslaunch basler_bridge basler_bridge.launch base_tof_name:=TOF0 base_tof_serial:=22218136 base_rgb_name:=RGB0 base_rgb_serial:=22410499 receiver:=true transmitter:=true calibrate:=true
```

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

##### RGB to World Extrinsic Calibration (Optional)

You can calibrate your new camera system to a fixed frame in the world. Fix your camera, somewhere, place your calibration board on the floor and call:

```
rosservice call /RGB0/Calibrate \
"frame: 'ws1_frame'
board_size: '9x6'
square_size: 0.117
direction: true
square_shift: 2
height_shift: -0.05
x_shift: 0.01
y_shift: 0.01
"
```
You can use the `/RGB0/calibration_preview` to see where your frame is. You can select your frame name, your board size, the size of the square in metres, the direction you want your TF Tree to publish in, which square you want your frame to be in, and a set of XYZ shifting to manually adjust and add offsets to the frame in case of errors. After calibration, your frame will be continuously published in a TF stream from the camera frame.

##### RGB to Robot Extrinsic Calibration (Optional)

This allows you to calibrate your robot end effector to the camera. This requires the camera and the ABB Robot Controller to be launched. Currently the parameters are hardcoded in the file.

```
roscd basler_bridge/scripts
python robot_calib_auto.py
# Choose Side or Bottom depending on the camera
```

### Running Camera

You can run your camera in several modes. To run all nodes locally on your system:

```
roslaunch basler_bridge basler_bridge.launch base_tof_name:=TOF0 base_tof_serial:=22100546 base_rgb_name:=RGB0 base_rgb_serial:=22099567 receiver:=true transmitter:=true
```

To run the RGB decompression and point cloud generation on one system and the Depth registration and RGB compression in one system: (To save bandwidth)

```
roslaunch basler_bridge basler_bridge.launch nodelet_manager:=RGB1_Receive base_tof_name:=TOF1 base_tof_serial:=22218136 base_rgb_name:=RGB1 base_rgb_serial:=22410499 receiver:=true transmitter:=false

roslaunch basler_bridge basler_bridge.launch nodelet_manager:=RGB1_Transmit base_tof_name:=TOF1 base_tof_serial:=22218136 base_rgb_name:=RGB1 base_rgb_serial:=22410499 receiver:=false transmitter:=true
```
To run only the RGB compression on one system and the rest in another: (In case the system connected closer to the camera cannot process depth fast enough)

```
roslaunch basler_bridge basler_bridge_compressrgb.launch nodelet_manager:=RGB1_Receive base_tof_name:=TOF1 base_tof_serial:=22218136 base_rgb_name:=RGB1 base_rgb_serial:=22410499 receiver:=true transmitter:=false

roslaunch basler_bridge basler_bridge_compressrgb.launch nodelet_manager:=RGB1_Transmit base_rgb_name:=RGB1 base_rgb_serial:=22410499 receiver:=false transmitter:=true
```

### Algorithms

https://github.com/rp5-tumcreate/ac2-ros-sensor-pkgs/blob/nuc-linux-devel/docs/tex/camera/Camera.pdf

https://github.com/rp5-tumcreate/ac2-ros-sensor-pkgs/blob/nuc-linux-devel/docs/tex/calibration/Calibration.pdf

### Contacts

```
Sales: 
11 Woodlands Close, #08-21, Woodlands 11, Singapore 737853
Office: (+65) 6694 0389
Mobile: (+65) 9457 7570
Email: melody.mok@sodavision.com
Website: www.sodavision.com

Support:
Jithin Sebastian
Application Engineer
Basler Asia Pte. Ltd.
35 Marsiling Industrial Estate Road 3
#05-06, Singapore 739257
Singapore
Tel. +65 63671355
Jithin.Sebastian@baslerweb.com
www.baslerweb.com
```
