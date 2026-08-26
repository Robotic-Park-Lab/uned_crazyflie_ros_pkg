# Copyright 2026 Robotic Park Lab
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Robotic Park Lab nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Resolución de las claves de configuración por-robot comunes.

Comunes a
Crazyflie_ROS2 (crazyflie_agent.py, robot físico) y CrazyflieWebotsDriver
(webots_driver.py, robot virtual en Webots).

Antes de esta extracción, ambas clases repetían la misma lectura de
config['control_mode'], config['controller']['type'], config['communication'],
config['local_pose'], etc. en su __init__/init() -- la versión física
accedía directamente (KeyError si faltaba una clave) y la de Webots
comprobaba "key" in config con valores por defecto. Se unifica aquí con el
estilo defensivo de Webots (superconjunto seguro: mismo resultado cuando
las claves están presentes, sin excepción cuando no lo están).

Deliberadamente NO incluye 'task' (formación): el punto en que cada clase
decide arrancar la lógica de formación difiere lo bastante entre ambas
(Crazyflie_ROS2.load_formation_params() se autolimita comprobando
config['task']['enable'] en varios sitios de crazyflie_agent.py;
CrazyflieWebotsDriver usa variables['']task_config/variables['']task_onboard como puerta
previa en initialize()) como para forzarlo aquí sin más riesgo del que
merece esta pasada -- ver AUDIT.md (rama doc) para el detalle.
"""

from std_msgs.msg import String, Bool, Float64, Float64MultiArray, MultiArrayDimension, UInt16MultiArray
from geometry_msgs.msg import Twist, Pose, Point, PoseStamped, Vector3
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path

def resolve_driver_config(config, node):
    resolved = {}

    resolved['name'] = config.get('name', 'dron00')
    resolved['positioning'] = config.get('positioning', 'Intern')
    resolved['physical'] = config.get('type') == 'physical'
    resolved['digital_twin'] = config.get('type') == 'digital_twin'

    # Controlador Interno
    resolved['control_mode'] = config.get('control_mode', 'HighLevel')
    node.info('Crazyflie %s::Control Mode: %s!' % (resolved['name'], resolved['control_mode']))
    resolved['controller_type'] = config.get('controller', {}).get('type', 'pid')
    resolved['eomas'] = 3.14
    resolved['controller_protocol'] = config.get('controller', {}).get('protocol', 'Continuous') == 'Continuous'
    node.info('Crazyflie %s::Controller Type: %s!' % (resolved['name'], resolved['controller_type']))

    # Comunicación
    communication = config.get('communication', {})
    resolved['communication'] = communication.get('type', 'Continuous') == 'Continuous'
    if resolved['communication']:
        resolved['threshold'] = 0.001
    else:
        resolved['threshold'] = communication['threshold']['co']

    # ROS 2 Params
    local_pose = config.get('local_pose', {})
    resolved['local_pose_enable'] = local_pose.get('enable', False)
    resolved['path_enable'] = local_pose.get('path', False)


    resolved['local_twist_enable'] = config.get('local_twist', {}).get('enable', False)
    resolved['data_attitude_enable'] = config.get('data_attitude', {}).get('enable', False)
    resolved['data_rate_enable'] = config.get('data_rate', {}).get('enable', False)
    resolved['data_motor_enable'] = config.get('data_motor', {}).get('enable', False)
    resolved['mars_data_enable'] = config.get('mars_data', {}).get('enable', False)
    resolved['data_enable'] = config.get('data', {}).get('enable', False)
    
    if "task" in config:
        task = config.get('task', {})
        resolved['task_type'] = task.get('type', 'distance')
        resolved['task_role'] = task.get('role', 'consensus')
        resolved['task_enable'] = task.get('enable', False)
        resolved['task_onboard'] = task.get('Onboard', False)
        if "controller" in task:
            controller = task.get('controller', {})
            resolved['task_contr_type'] = controller.get('type', 'gradient')
            resolved['task_contr_protocol'] = controller.get('protocol', 'Continuous') == 'Continuous'
            resolved['task_contr_period'] = controller.get('period', 0.1)
            resolved['task_contr_ul'] = controller.get('upperLimit', 0.1)
            resolved['task_contr_ll'] = controller.get('lowerLimit', -0.1)
            resolved['task_contr_gain'] = controller.get('gain', 0.05)
            threshold = controller.get('threshold', {})
            resolved['task_contr_th_ai'] = threshold.get('ai', 0.05)
            resolved['task_contr_th_co'] = threshold.get('co', 0.05)
        

    return resolved

def resolve_ros2_config(robot):
    publisher = {}
    subscriber = {}

    # POSE3D
    if robot.driver_cfg['local_pose_enable']:
        if robot.driver_cfg['path_enable']:
            publisher['path'] = robot.node.create_publisher(Path, robot.id + '/path', 10)
        if robot.driver_cfg['digital_twin']:
            pose_name = robot.id + '/dt_pose'
            subscriber['dt_pose'] = robot.node.create_subscription(PoseStamped, robot.id + '/local_pose', robot.dt_pose_callback, 1)
        else:
            pose_name = robot.id + '/local_pose'
        publisher['pose_publisher'] = robot.node.create_publisher(PoseStamped, pose_name, 10)
        publisher['pose_publisher_gt'] = robot.node.create_publisher(PoseStamped, pose_name + '_gt', 10)
    # TWIST
    if robot.driver_cfg['local_twist_enable']:
        publisher['twist']  = robot.node.create_publisher(Twist, robot.id + '/local_twist', 10)
    
    # DATA ATTITUDE.
    if robot.driver_cfg['data_attitude_enable']:
        publisher['attitude'] = robot.node.create_publisher(Float64MultiArray, robot.id + '/data_attitude', 10)
    
    # DATA RATE.
    if robot.driver_cfg['data_rate_enable']:
        publisher['rate'] = robot.node.create_publisher(Float64MultiArray, robot.id + '/data_rate', 10)
    
    # DATA MOTOR.
    if robot.driver_cfg['data_motor_enable']:
        publisher['motors'] = robot.node.create_publisher(Float64MultiArray, robot.id + '/data_motor', 10)
    
    # MULTIROBOT
    if robot.driver_cfg['mars_data_enable']:
        publisher['mrs_data'] = robot.node.create_publisher(Float64MultiArray, robot.id + '/mr_data', 10)
        publisher['mrs_data_mod'] = robot.node.create_publisher(Float64, robot.id + '/mr_data_mod', 10)
        publisher['mrs_data_gt'] = robot.node.create_publisher(Float64MultiArray, robot.id + '/mr_data_gt', 10)
        publisher['mrs_data_gt_mod'] = robot.node.create_publisher(Float64, robot.id + '/mr_data_gt_mod', 10)
    
    # DATA.
    if robot.driver_cfg['data_enable']:
        publisher['data'] = robot.node.create_publisher(UInt16MultiArray, robot.id + '/data', 10)
    # if not robot.driver_cfg['communication']:
    publisher['event_x'] = robot.node.create_publisher(Bool, robot.id + '/event_x', 10)
    publisher['event_y'] = robot.node.create_publisher(Bool, robot.id + '/event_y', 10)
    publisher['event_z'] = robot.node.create_publisher(Bool, robot.id + '/event_z', 10)
    # Subscription
    subscriber['cmd_vel'] = robot.node.create_subscription(Twist, robot.id + '/cmd_vel', robot.cmd_vel_callback, 1)
    subscriber['goal_pose'] = robot.node.create_subscription(PoseStamped, robot.id + '/goal_pose', robot.goal_pose_callback, 1)
    subscriber['target_pose'] = robot.node.create_subscription(PoseStamped, robot.id + '/target_pose', robot.goal_pose_callback, 1)
    subscriber['order'] = robot.node.create_subscription(String, robot.id + '/order', robot.order_callback, 1)
    subscriber['swarm_order'] = robot.node.create_subscription(String, 'swarm/order', robot.order_callback, 1)
    subscriber['swarm_goal_pose'] = robot.node.create_subscription(PoseStamped, 'swarm/goal_pose', robot.swarm_goalpose_callback, 1)
    # Publisher
    publisher['laser'] = robot.node.create_publisher(LaserScan, robot.id + '/scan', 10)
    publisher['swarm_status'] = robot.node.create_publisher(String, 'swarm/status', 10)
    publisher['odom'] = robot.node.create_publisher(Odometry, robot.id + '/odom', 10)

    
    return publisher, subscriber

def resolve_variable_config():
    variables = {}

    variables['state'] = [10.0, 10.0, 10.0, 10.0, 10.0]                 # Reconfiguration
    variables['update_gain'] = True                                     # Reconfiguration
    variables['ready'] = False
    variables['disconnect'] = False
    variables['swarm_ready'] = False
    
    variables['controller_type'] = 'gradient_p'
    # RELAY
    variables['rele_cmd_eq'] = 0.0
    variables['relay_level'] = False
    # Position level
    variables['relay_p_cmd'] = 0.2
    variables['relay_p_threshold'] = 0.02
    variables['rele_p'] = False
    variables['rele_x'] = False
    variables['rele_y'] = False
    variables['rele_z'] = False
    # Speed level
    variables['relay_s_cmd'] = 4.0
    variables['relay_s_threshold'] = 0.05
    variables['rele_s'] = False
    # Attitude level
    variables['relay_a_cmd'] = 15.0
    variables['relay_a_threshold'] = 0.2
    variables['rele_a'] = False
    # Rate level
    variables['relay_r_cmd'] = 0.0
    variables['relay_r_threshold'] = 0.0
    variables['rele_r'] = False
    
    variables['target_twist'] = Twist()
    variables['target_pose'] = PoseStamped()
    variables['target_pose'].header.frame_id = 'map'
    variables['pose'] = Pose()
    variables['last_pose'] = Pose()
    variables['home'] = Pose()
    variables['path'] = Path()
    variables['path'].header.frame_id = 'map'
    variables['past_x_global'] = 0
    variables['past_y_global'] = 0
    variables['past_z_global'] = 0

    variables['_is_flying'] = False
    variables['init_pose'] = False
    variables['formation'] = False
    variables['distance_formation_bool_update'] = True
    variables['N'] = 1.0

    variables['centroid_leader'] = False
    variables['leader_cmd'] = PoseStamped()
    variables['leader_cmd'].header.frame_id = 'map'
    
    variables['trigger_ai'] = 0.01
    variables['trigger_co'] = 0.1
    variables['trigger_last_signal'] = 0.0

    return variables