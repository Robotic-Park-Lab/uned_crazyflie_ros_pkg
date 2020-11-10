#!/bin/bash
echo "Extraer datos .bag a .txt"
echo "-------------------------"

file=$1
ID_dataset=$2

mkdir $ID_dataset

echo "Topic (1/32): /clock"
echo "..."
aux="_clock"
rostopic echo -b $file -p /clock >> $ID_dataset/$ID_dataset$aux.txt

echo "Topic (2/32): /crazyflie2/command/motor_speed"
echo "..."
aux="_Cf2_cmd_mspeed"
rostopic echo -b $file -p /crazyflie2/command/motor_speed >> $ID_dataset/$ID_dataset$aux.txt

echo "Topic (3/32): /crazyflie2/gazebo/command/motor_speed "
echo "..."
aux="_Cf2_gaz_cmd_mspeed"
rostopic echo -b $file -p /crazyflie2/gazebo/command/motor_speed >> $ID_dataset/$ID_dataset$aux.txt

echo "Topic (4/32): /crazyflie2/ground_truth/imu"
echo "..."
aux="_Cf2_gt_imu"
rostopic echo -b $file -p /crazyflie2/ground_truth/imu >> $ID_dataset/$ID_dataset$aux.txt

echo "Topic (5/32): /crazyflie2/ground_truth/odometry"
echo "..."
aux="_Cf2_gt_odom"
rostopic echo -b $file -p /crazyflie2/ground_truth/odometry >> $ID_dataset/$ID_dataset$aux.txt

echo "Topic (6/32): /crazyflie2/ground_truth/pose"
echo "..."
aux="_Cf2_gt_pose"
rostopic echo -b $file -p /crazyflie2/ground_truth/pose >> $ID_dataset/$ID_dataset$aux.txt

echo "Topic (7/32): /crazyflie2/ground_truth/pose_with_covariance"
echo "..."
aux="_Cf2_gt_posecov"
rostopic echo -b $file -p /crazyflie2/ground_truth/pose_with_covariance >> $ID_dataset/$ID_dataset$aux.txt

echo "Topic (8/32): /crazyflie2/ground_truth/position"
echo "..."
aux="_Cf2_gt_pos"
rostopic echo -b $file -p /crazyflie2/ground_truth/position >> $ID_dataset/$ID_dataset$aux.txt

echo "Topic (9/32): /crazyflie2/ground_truth/transform "
echo "..."
aux="_Cf2_gt_transf"
rostopic echo -b $file -p /crazyflie2/ground_truth/transform >> $ID_dataset/$ID_dataset$aux.txt

echo "Topic (10/32): /crazyflie2/joint_states"
echo "..."
aux="_Cf2_js"
rostopic echo -b $file -p /crazyflie2/joint_states >> $ID_dataset/$ID_dataset$aux.txt

echo "Topic (11/32): /crazyflie2/motor_speed"
echo "..."
aux="_Cf2_mspeed"
rostopic echo -b $file -p /crazyflie2/motor_speed >> $ID_dataset/$ID_dataset$aux.txt

echo "Topic (12/32): /crazyflie2/odometry"
echo "..."
aux="_Cf2_odom"
rostopic echo -b $file -p /crazyflie2/odometry >> $ID_dataset/$ID_dataset$aux.txt

echo "Topic (13/32): /crazyflie2/odometry_sensor1/odometry"
echo "..."
aux="_Cf2_odoms1_odom"
rostopic echo -b $file -p /crazyflie2/odometry_sensor1/odometry >> $ID_dataset/$ID_dataset$aux.txt

echo "Topic (14/32): /crazyflie2/odometry_sensor1/pose"
echo "..."
aux="_Cf2_odoms1_pose"
rostopic echo -b $file -p /crazyflie2/odometry_sensor1/pose >> $ID_dataset/$ID_dataset$aux.txt

echo "Topic (15/32): /crazyflie2/odometry_sensor1/pose_with_covariance"
echo "..."
aux="_Cf2_odoms1_posecov"
rostopic echo -b $file -p /crazyflie2/odometry_sensor1/pose_with_covariance >> $ID_dataset/$ID_dataset$aux.txt

echo "Topic (16/32): /crazyflie2/odometry_sensor1/position"
echo "..."
aux="_Cf2_odoms1_pos"
rostopic echo -b $file -p /crazyflie2/odometry_sensor1/position >> $ID_dataset/$ID_dataset$aux.txt

echo "Topic (17/32): /crazyflie2/odometry_sensor1/transform "
echo "..."
aux="_Cf2_odoms1_transf"
rostopic echo -b $file -p /crazyflie2/odometry_sensor1/transform >> $ID_dataset/$ID_dataset$aux.txt

echo "Topic (18/32): /crazyflie2/orientation_rpy"
echo "..."
aux="_Cf2_rpy"
rostopic echo -b $file -p /crazyflie2/orientation_rpy >> $ID_dataset/$ID_dataset$aux.txt

echo "Topic (19/32): /crazyflie2/pose"
echo "..."
aux="_Cf2_pose"
rostopic echo -b $file -p /crazyflie2/pose >> $ID_dataset/$ID_dataset$aux.txt

echo "Topic (20/32): /crazyflie2/pose_with_covariance"
echo "..."
aux="_Cf2_posecov"
rostopic echo -b $file -p /crazyflie2/pose_with_covariance >> $ID_dataset/$ID_dataset$aux.txt

echo "Topic (21/32): /crazyflie2/position"
echo "..."
aux="_Cf2_pos"
rostopic echo -b $file -p /crazyflie2/position >> $ID_dataset/$ID_dataset$aux.txt

echo "Topic (22/32): /crazyflie2/transform"
echo "..."
aux="_Cf2_transf"
rostopic echo -b $file -p /crazyflie2/transform >> $ID_dataset/$ID_dataset$aux.txt

echo "Topic (23/32): /gazebo/link_states"
echo "..."
aux="_gaz_lstates"
rostopic echo -b $file -p /gazebo/link_states >> $ID_dataset/$ID_dataset$aux.txt

echo "Topic (24/32): /gazebo/model_states"
echo "..."
aux="_gaz_mstates"
rostopic echo -b $file -p /gazebo/model_states >> $ID_dataset/$ID_dataset$aux.txt

echo "Topic (25/32): /gazebo/parameter_descriptions"
echo "..."
aux="_gaz_param"
rostopic echo -b $file -p /gazebo/parameter_descriptions >> $ID_dataset/$ID_dataset$aux.txt

echo "Topic (26/32): /gazebo/parameter_updates"
echo "..."
aux="_gaz_paramupdate"
rostopic echo -b $file -p /gazebo/parameter_updates >> $ID_dataset/$ID_dataset$aux.txt

echo "Topic (27/32): /tf"
echo "..."
aux="_tf"
rostopic echo -b $file -p /tf >> $ID_dataset/$ID_dataset$aux.txt

echo "Topic (28/32): /tf_static"
echo "..."
aux="_tf_static"
rostopic echo -b $file -p /tf_static >> $ID_dataset/$ID_dataset$aux.txt

echo "Proceso Finalizado"
