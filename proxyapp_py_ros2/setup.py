from setuptools import setup
package_name = 'proxyapp_py_ros2'

setup(
    name=package_name,
    version='0.0.1',
    packages=['proxyapp_py_ros2'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mehmet',
    maintainer_email='mehmet.killioglu@unikie.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    keywords=['ROS'],
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'unsecure_client = proxyapp_py_ros2.unsecure_client:main',
            'secure_server = proxyapp_py_ros2.secure_server:main'
        ],
    },
)
