from glob import glob

from setuptools import setup

package_name = 'grasp_monitoring'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lorenzo',
    maintainer_email='ghessi2003@gmail.com',
    description='Standalone geometric grasp monitor (target vs end-effector pose).',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'geometric_grasp_monitor = '
            'grasp_monitoring.geometric_grasp_monitor_node:main',
        ],
    },
)
