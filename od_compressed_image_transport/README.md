## On Demand Compressed Image Transport

This package compresses and decompresses images on demand, depending if you are subscribed to the topic or not. This is hence a better extension of the the `image_transport` compression package.

### Usage

This is an example of compression and decompress

```
<launch>

    <!-- Args -->
    <arg name="queue_size"        default="5"/>
    <arg name="num_worker_threads"    default="4"/>
    <arg name="machine"           default="localhost"/>
    <arg name="nodelet_manager"   default="basler_bridge"/>
    <arg name="start_manager"     default="true"/>
    <arg name="use_machine"       default="true"/>
    <arg name="respawn"           default="false"/>
    <arg name="use_nodelet"       default="true"/>
    <arg name="calibrate"         default="false"/>

    <!-- Nodelet Manager -->
    <machine name="localhost" address="localhost" if="$(arg use_machine)"/>
    <node pkg="nodelet" type="nodelet" name="$(arg nodelet_manager)" args="manager"
    if="$(arg start_manager)" machine="$(arg machine)" output="screen">
    <param name="num_worker_threads" value="$(arg num_worker_threads)" />
    </node>

    <!-- Publisher -->
    <node pkg="nodelet" type="nodelet" name="compressed_publisher" machine="$(arg machine)"
    args="load od_compressed_image_transport/compressed_publisher $(arg nodelet_manager)" respawn="$(arg respawn)">
        <param name="queue_size" type="int" value="$(arg queue_size)"/>
        <param name="raw_topic_input" type="string" value="/Kinect1/hd/image_color_rect"/>
        <param name="compressed_topic_output" type="string" value="/Kinect1/hd/image_color_rect/compressed"/>
        <param name="retime" type="bool" value="false"/>
    </node>
    
    <!-- Subscriber -->
    <node pkg="nodelet" type="nodelet" name="compressed_subscriber" machine="$(arg machine)"
    args="load od_compressed_image_transport/compressed_subscriber $(arg nodelet_manager)" respawn="$(arg respawn)">
        <param name="queue_size" type="int" value="$(arg queue_size)"/>
        <param name="compressed_topic_input" type="string" value="/Kinect1/hd/image_color_rect/compressed"/>
        <param name="raw_topic_output" type="string" value="/Kinect1/hd/image_color_rect"/>
        <param name="retime" type="bool" value="false"/>
    </node>
    
</launch>
```

The `retime` flag should be set to false by default. Unless you want the time of the data to be reset to a new ros::Time::now().
