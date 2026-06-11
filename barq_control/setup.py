from setuptools import find_packages, setup

package_name = 'barq_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ik_node = barq_control.ik_node:main',
            'gait_planner = barq_control.gait_planner_node:main',
            'odom_tf = barq_control.odom_tf_node:main',
            'state_estimator = barq_control.state_estimator_node:main',
        ],
    },
)
