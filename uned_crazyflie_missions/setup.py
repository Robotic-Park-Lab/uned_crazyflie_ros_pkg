from setuptools import setup

package_name = 'uned_crazyflie_missions'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Francisco José Mañas Álvarez',
    maintainer_email='fjmanas@dia.uned.es',
    description='Nodos de misión de alto nivel para el crazyflie 2.1, indiferentes a si el dron'
    ' es físico o virtual: formaciones (leader-follower, basada en forma, en Webots) y'
    ' recorrido de waypoints tipo TSP',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'leader_follower = uned_crazyflie_missions.leader_follower:main',
            'shape_based_formation_control ='
            ' uned_crazyflie_missions.shape_based_formation_control:main',
            'formation_control_webots = uned_crazyflie_missions.formation_control_webots:main',
            'tsp_waypoints = uned_crazyflie_missions.tsp_waypoints:main',
        ],
    },
)
