
## Cloud Generator

The cloud generator nodelet takes in an RGB Image, Registered Depth Image, RGB Camera Matrix and Registered Depth Camera Matrix and a RGB to World frame through TF to generate a point cloud on the GPU. Internally, it also runs a temporal filter on the depth map to remove outliers and smooth it out.

### Usage

```
<!-- cloud generator -->
<node pkg="nodelet" type="nodelet" name="$(arg base_rgb_name)_cloud_generator" machine="$(arg machine)"
args="load cloud_generator/cloud_generator $(arg nodelet_manager)" respawn="$(arg respawn)">
<param name="queue_size" type="int" value="$(arg queue_size)"/>
<param name="temporal_filter" type="bool" value="true"/>
<param name="temporal_error" type="double" value="0.1"/>
<param name="unorganize" type="bool" value="true"/>
<param name="republish" type="bool" value="true"/>
<param name="tf_target" type="string" value="$(arg base_rgb_name)_rgb_optical_frame"/>
<param name="sub_depth_topic" type="string" value="$(arg base_rgb_name)/registered/depth_rect"/>
<param name="sub_rgb_topic" type="string" value="$(arg base_rgb_name)/image_color_rect/republished"/>
<param name="sub_depth_info_topic" type="string" value="$(arg base_rgb_name)/registered/camera_info"/>
<param name="sub_rgb_info_topic" type="string" value="$(arg base_rgb_name)/camera_info"/>
<param name="pub_edge_topic" type="string" value="$(arg base_rgb_name)/edge_rect"/>
<param name="pub_cloud_topic" type="string" value="$(arg base_rgb_name)/registered/points"/>
<param name="pub_depth_republish_topic" type="string" value="$(arg base_rgb_name)/registered/depth_rect/republished"/>
<param name="pub_rgb_republish_topic" type="string" value="$(arg base_rgb_name)/registered/image_color_rect/republished"/>
</node>
```

This is to be used in conjunction with a nodelet manager. See Basler Bridge code for usage example. The `temporal_filter` and `temporal_error` flag perform post processing on the depth map to patch up holes, remove outliers and smooth out data. The `unorganize` flag disorganizes the point data. It loses spatial information, but can save significant bandwidth. `republish` republishes some of the depth and rgb information in case processing has been applied on it. The data is republished to the `pub_depth_republish_topic` and `pub_rgb_republish_topic` topics but this is optional. `tf_target` is the TF name of the transform you wish your data to be transformed to a particular frame on the GPU. This can save some processing later. By default it is set to the camera frame which would produce an identity transform.  `sub_depth_topic`, `sub_rgb_topic` are the depth and rgb image topics, and `sub_depth_info_topic`, `sub_rgb_info_topic` are the depth and rgb camera matrices.  `pub_edge_topic` is the edge image topic which is currently not in use but needs to be set, and `pub_cloud_topic` is the point cloud topic.

### Algorithm

More information on how the temporal depth processing occurs and how the depth map is converted to a point cloud can be found [here](https://github.com/rp5-tumcreate/ac2-ros-sensor-pkgs/blob/nuc-linux-devel/docs/tex/camera/Camera.pdf).

