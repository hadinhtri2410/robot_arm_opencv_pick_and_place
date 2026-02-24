import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'opencv_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='triha',
    maintainer_email='hai49@purdue.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'load_images_from_camera = opencv_perception.load_images_from_camera:main',
            'color_filtering = opencv_perception.color_filtering:main',
            'edge_detection = opencv_perception.edge_detection:main',
            'stream_camera = opencv_perception.stream_camera:main',
            'hsv_from_mouse = opencv_perception.hsv_from_mouse:main',
            'load_images = opencv_perception.load_images:main',
        ],
    },
)
