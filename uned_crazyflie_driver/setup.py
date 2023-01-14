from setuptools import setup

package_name = 'uned_crazyflie_driver'

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
    description='Paquete para el manejo de los nanodrones crazyflie 2.1',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'crazyflie_driver = uned_crazyflie_driver.crazyflie_driver:main',
            'swarm_driver = uned_crazyflie_driver.swarm_driver:main',
            'positioning_system = uned_crazyflie_driver.positioning_system:main'
        ],
    },
)
