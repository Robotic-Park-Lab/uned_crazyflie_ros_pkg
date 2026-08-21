from setuptools import setup

package_name = 'uned_crazyflie_common'

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
    description='Código Python compartido entre uned_crazyflie_driver, uned_crazyflie_webots y'
    ' uned_crazyflie_task: controlador PID y aplicación de parámetros PID del Crazyflie',
    license='BSD-3-Clause',
    tests_require=['pytest'],
)
