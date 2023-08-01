from setuptools import setup

package_name = 'my_robot_controller'

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
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # executable = pkg.file_name:function_want_to_run
            "turtle_test_publish = my_robot_controller.turtle_test_publish:main",
            "test_subscriber = my_robot_controller.test_subscriber:main",
            "test_driver = my_robot_controller.test_driver:main",
            "odom_data_subscriber = my_robot_controller.odom_data_subscriber:main",
            "camera_simulator = my_robot_controller.camera_pub_sim:main",
            "camera_data_reader = my_robot_controller.camera_data_reader:main",
            "transformation_test = my_robot_controller.transformation_test:main",
            "tf_static_extractor = my_robot_controller.tf_static_extractor:main"
        ],
    },
)
