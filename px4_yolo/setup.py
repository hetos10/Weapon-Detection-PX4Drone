from setuptools import find_packages, setup

package_name = 'px4_yolo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    py_modules=[
        'yolo_detector_node'
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hetos_10',
    maintainer_email='hetchauhan22@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_detector = yolo_detector_node:main'

        ],
    },
)
