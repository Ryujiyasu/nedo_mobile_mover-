from setuptools import setup

package_name = 'nedo_mobile_mover'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, [
            'package.xml',

            ]),
        ('share/' + package_name + '/launch/', [
            'launch/mm.state.publisher.py',
            'launch/robot.launch.py',
            ]),
        ('share/' + package_name + '/urdf/', [
            'urdf/nedo_mobile_mover.urdf',
            ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ryuji Yasukochi',
    maintainer_email='ryuji.yasu@gmail.com',
    description='controller for nedo mobile mover',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main = nedo_mobile_mover.main:main'
        ],
    },
)
