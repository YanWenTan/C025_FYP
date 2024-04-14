from setuptools import find_packages, setup

package_name = 'cv_ros2_test'

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
    maintainer='yanwen',
    maintainer_email='yanwen@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                            'obj_detection_publisher = cv_ros2_test.object_detection_publisher:main',
                            'transform_pose_publisher = cv_ros2_test.object_pose_transform:main'
        ],
    },
)
