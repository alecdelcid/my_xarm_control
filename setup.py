from setuptools import find_packages, setup

package_name = 'my_xarm_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/my_xarm_demo.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Custom Python scripts for xArm control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'basic_movement = my_xarm_control.basic_movement:main',
            'moveit_example = my_xarm_control.moveit_example:main',
            'gripper_control = my_xarm_control.gripper_control:main',
        ],
    },
)