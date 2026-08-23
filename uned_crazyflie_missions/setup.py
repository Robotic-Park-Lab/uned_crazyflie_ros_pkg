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
    description='High-level mission nodes for the crazyflie 2.1: each one talks only over'
    ' topics (never to cflib/Webots directly), so the same node works with a physical or'
    ' virtual drone -- formation control, waypoint touring, and generic topic sequencing,'
    ' all configured from a .yaml, not hardcoded',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'formation = uned_crazyflie_missions.formation:main',
            'waypoints = uned_crazyflie_missions.waypoints:main',
            'sequencer = uned_crazyflie_missions.sequencer:main',
        ],
    },
)
