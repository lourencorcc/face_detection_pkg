from setuptools import find_packages, setup
from glob import glob
package_name = 'face_detection_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', [
            'config/yolov3.weights',
            'config/yolov3.cfg',
            'config/coco.names',
            'config/haarcascade_frontalface_default.xml',
        ]),
        ('share/' + package_name + '/launch', glob('launch/*launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lourencorcc',
    maintainer_email='lourencorconceicao@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'face_detection_node = face_detection_pkg.face_detection_node:main',
            'viewer_node = face_detection_pkg.viewer_node:main',
            'debug_node = face_detection_pkg.debug_node:main',
            'follow_human = face_detection_pkg.follow_human:main'
        ],
    },
)
