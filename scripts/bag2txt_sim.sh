#!/bin/bash
echo "Extraer datos .bag a .txt"
echo "-------------------------"

file=$1
ID_dataset=$2
if !(test -d "$ID_dataset")
then
mkdir $ID_dataset
else
echo "Carpeta existente"
fi

echo "Topic (1/28): /clock"
echo "..."
aux="_clock"
if !(test -f "$ID_dataset/$ID_dataset$aux.txt")
then
rostopic echo -b $file -p /clock > $ID_dataset/$ID_dataset$aux.txt
else
echo "Archivo existente"
fi

echo "Topic (2/28): /crazyflie2/command/motor_speed"
echo "..."
aux="_Cf2_cmd_mspeed"
if !(test -f "$ID_dataset/$ID_dataset$aux.txt")
then
rostopic echo -b $file -p /crazyflie2/command/motor_speed > $ID_dataset/$ID_dataset$aux.txt
else
echo "Archivo existente"
fi

echo "Topic (3/28): /crazyflie2/gazebo/command/motor_speed "
echo "..."
aux="_Cf2_gaz_cmd_mspeed"
if !(test -f "$ID_dataset/$ID_dataset$aux.txt")
then
rostopic echo -b $file -p /crazyflie2/gazebo/command/motor_speed > $ID_dataset/$ID_dataset$aux.txt
else
echo "Archivo existente"
fi

echo "Topic (4/28): /crazyflie2/ground_truth/imu"
echo "..."
aux="_Cf2_gt_imu"
if !(test -f "$ID_dataset/$ID_dataset$aux.txt")
then
rostopic echo -b $file -p /crazyflie2/ground_truth/imu > $ID_dataset/$ID_dataset$aux.txt
else
echo "Archivo existente"
fi

echo "Topic (5/28): /crazyflie2/ground_truth/odometry"
echo "..."
aux="_Cf2_gt_odom"
if !(test -f "$ID_dataset/$ID_dataset$aux.txt")
then
rostopic echo -b $file -p /crazyflie2/ground_truth/odometry > $ID_dataset/$ID_dataset$aux.txt
else
echo "Archivo existente"
fi

echo "Topic (6/28): /crazyflie2/ground_truth/pose"
echo "..."
aux="_Cf2_gt_pose"
if !(test -f "$ID_dataset/$ID_dataset$aux.txt")
then
rostopic echo -b $file -p /crazyflie2/ground_truth/pose > $ID_dataset/$ID_dataset$aux.txt
else
echo "Archivo existente"
fi

echo "Topic (7/28): /crazyflie2/ground_truth/pose_with_covariance"
echo "..."
aux="_Cf2_gt_posecov"
if !(test -f "$ID_dataset/$ID_dataset$aux.txt")
then
rostopic echo -b $file -p /crazyflie2/ground_truth/pose_with_covariance > $ID_dataset/$ID_dataset$aux.txt
else
echo "Archivo existente"
fi

echo "Topic (8/28): /crazyflie2/ground_truth/position"
echo "..."
aux="_Cf2_gt_pos"
if !(test -f "$ID_dataset/$ID_dataset$aux.txt")
then
rostopic echo -b $file -p /crazyflie2/ground_truth/position > $ID_dataset/$ID_dataset$aux.txt
else
echo "Archivo existente"
fi

echo "Topic (9/28): /crazyflie2/ground_truth/transform "
echo "..."
aux="_Cf2_gt_transf"
if !(test -f "$ID_dataset/$ID_dataset$aux.txt")
then
rostopic echo -b $file -p /crazyflie2/ground_truth/transform > $ID_dataset/$ID_dataset$aux.txt
else
echo "Archivo existente"
fi

echo "Topic (10/28): /crazyflie2/joint_states"
echo "..."
aux="_Cf2_js"
if !(test -f "$ID_dataset/$ID_dataset$aux.txt")
then
rostopic echo -b $file -p /crazyflie2/joint_states > $ID_dataset/$ID_dataset$aux.txt
else
echo "Archivo existente"
fi

echo "Topic (11/28): /crazyflie2/motor_speed"
echo "..."
aux="_Cf2_mspeed"
if !(test -f "$ID_dataset/$ID_dataset$aux.txt")
then
rostopic echo -b $file -p /crazyflie2/motor_speed > $ID_dataset/$ID_dataset$aux.txt
else
echo "Archivo existente"
fi

echo "Topic (12/28): /crazyflie2/odometry"
echo "..."
aux="_Cf2_odom"
if !(test -f "$ID_dataset/$ID_dataset$aux.txt")
then
rostopic echo -b $file -p /crazyflie2/odometry > $ID_dataset/$ID_dataset$aux.txt
else
echo "Archivo existente"
fi

echo "Topic (13/28): /crazyflie2/odometry_sensor1/odometry"
echo "..."
aux="_Cf2_odoms1_odom"
if !(test -f "$ID_dataset/$ID_dataset$aux.txt")
then
rostopic echo -b $file -p /crazyflie2/odometry_sensor1/odometry > $ID_dataset/$ID_dataset$aux.txt
else
echo "Archivo existente"
fi

echo "Topic (14/28): /crazyflie2/odometry_sensor1/pose"
echo "..."
aux="_Cf2_odoms1_pose"
if !(test -f "$ID_dataset/$ID_dataset$aux.txt")
then
rostopic echo -b $file -p /crazyflie2/odometry_sensor1/pose > $ID_dataset/$ID_dataset$aux.txt
else
echo "Archivo existente"
fi

echo "Topic (15/28): /crazyflie2/odometry_sensor1/pose_with_covariance"
echo "..."
aux="_Cf2_odoms1_posecov"
if !(test -f "$ID_dataset/$ID_dataset$aux.txt")
then
rostopic echo -b $file -p /crazyflie2/odometry_sensor1/pose_with_covariance > $ID_dataset/$ID_dataset$aux.txt
else
echo "Archivo existente"
fi

echo "Topic (16/28): /crazyflie2/odometry_sensor1/position"
echo "..."
aux="_Cf2_odoms1_pos"
if !(test -f "$ID_dataset/$ID_dataset$aux.txt")
then
rostopic echo -b $file -p /crazyflie2/odometry_sensor1/position > $ID_dataset/$ID_dataset$aux.txt
else
echo "Archivo existente"
fi

echo "Topic (17/28): /crazyflie2/odometry_sensor1/transform "
echo "..."
aux="_Cf2_odoms1_transf"
if !(test -f "$ID_dataset/$ID_dataset$aux.txt")
then
rostopic echo -b $file -p /crazyflie2/odometry_sensor1/transform > $ID_dataset/$ID_dataset$aux.txt
else
echo "Archivo existente"
fi

echo "Topic (18/28): /crazyflie2/orientation_rpy"
echo "..."
aux="_Cf2_rpy"
if !(test -f "$ID_dataset/$ID_dataset$aux.txt")
then
rostopic echo -b $file -p /crazyflie2/orientation_rpy > $ID_dataset/$ID_dataset$aux.txt
else
echo "Archivo existente"
fi

echo "Topic (19/28): /crazyflie2/pose"
echo "..."
aux="_Cf2_pose"
if !(test -f "$ID_dataset/$ID_dataset$aux.txt")
then
rostopic echo -b $file -p /crazyflie2/pose > $ID_dataset/$ID_dataset$aux.txt
else
echo "Archivo existente"
fi

echo "Topic (20/28): /crazyflie2/pose_with_covariance"
echo "..."
aux="_Cf2_posecov"
if !(test -f "$ID_dataset/$ID_dataset$aux.txt")
then
rostopic echo -b $file -p /crazyflie2/pose_with_covariance > $ID_dataset/$ID_dataset$aux.txt
else
echo "Archivo existente"
fi

echo "Topic (21/28): /crazyflie2/position"
echo "..."
aux="_Cf2_pos"
if !(test -f "$ID_dataset/$ID_dataset$aux.txt")
then
rostopic echo -b $file -p /crazyflie2/position > $ID_dataset/$ID_dataset$aux.txt
else
echo "Archivo existente"
fi

echo "Topic (22/28): /crazyflie2/transform"
echo "..."
aux="_Cf2_transf"
if !(test -f "$ID_dataset/$ID_dataset$aux.txt")
then
rostopic echo -b $file -p /crazyflie2/transform > $ID_dataset/$ID_dataset$aux.txt
else
echo "Archivo existente"
fi

echo "Topic (23/28): /gazebo/link_states"
echo "..."
aux="_gaz_lstates"
if !(test -f "$ID_dataset/$ID_dataset$aux.txt")
then
rostopic echo -b $file -p /gazebo/link_states > $ID_dataset/$ID_dataset$aux.txt
else
echo "Archivo existente"
fi

echo "Topic (24/28): /gazebo/model_states"
echo "..."
aux="_gaz_mstates"
if !(test -f "$ID_dataset/$ID_dataset$aux.txt")
then
rostopic echo -b $file -p /gazebo/model_states > $ID_dataset/$ID_dataset$aux.txt
else
echo "Archivo existente"
fi

echo "Topic (25/28): /gazebo/parameter_descriptions"
echo "..."
aux="_gaz_param"
if !(test -f "$ID_dataset/$ID_dataset$aux.txt")
then
rostopic echo -b $file -p /gazebo/parameter_descriptions > $ID_dataset/$ID_dataset$aux.txt
else
echo "Archivo existente"
fi

echo "Topic (26/28): /gazebo/parameter_updates"
echo "..."
aux="_gaz_paramupdate"
if !(test -f "$ID_dataset/$ID_dataset$aux.txt")
then
rostopic echo -b $file -p /gazebo/parameter_updates > $ID_dataset/$ID_dataset$aux.txt
else
echo "Archivo existente"
fi

echo "Topic (27/28): /tf"
echo "..."
aux="_tf"
if !(test -f "$ID_dataset/$ID_dataset$aux.txt")
then
rostopic echo -b $file -p /tf > $ID_dataset/$ID_dataset$aux.txt
else
echo "Archivo existente"
fi

echo "Topic (28/28): /tf_static"
echo "..."
aux="_tf_static"
if !(test -f "$ID_dataset/$ID_dataset$aux.txt")
then
rostopic echo -b $file -p /tf_static > $ID_dataset/$ID_dataset$aux.txt
else
echo "Archivo existente"
fi

echo "Proceso Finalizado"
