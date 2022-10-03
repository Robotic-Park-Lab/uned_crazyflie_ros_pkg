from setuptools import setup

package_name = 'uned_crazyflie_task'

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
    maintainer='kiko',
    maintainer_email='fjmanas@dia.uned.es',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'leader_follower = uned_crazyflie_task.leader_follower:main',
            'shape_based_formation_control = uned_crazyflie_task.shape_based_formation_control:main',
            'formation_control_webots = uned_crazyflie_task.formation_control_webots:main'
            
        ],
    },
)
