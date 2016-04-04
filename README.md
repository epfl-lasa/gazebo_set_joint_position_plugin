# gazebo_set_joint_position_plugin
Gazebo model plugin to set joints position on a robot (useful when replaying a rosbag for instance)

Example usage: 
add this piece of code in your robot's model file:


    <gazebo>
        <plugin name="set_joint_position" filename="libset_joint_position_plugin.so">
            <topicName> /set_gazebo_joint_states </topicName>
        </plugin>
    </gazebo>
    
    
You can set *topicName* to the name of the topic of the rosbag file you want to replay, or remap it as in this example:

    rosbag play /allegro_kuka/joint_states:=/set_gazebo_joint_states YOURBAGFILE
