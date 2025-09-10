from setuptools import setup

package_name = 'my_pub_sub'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Publisher-Subscriber package for ROS2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'random_five_publisher = my_pub_sub.random_five_publisher:main',
            'random_five_subscriber = my_pub_sub.random_five_subscriber:main',
        ],
    },
)
