from setuptools import setup
from glob import glob
import os

package_name = 'rosgpt'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, "prompts"), glob('prompts/*')),
        # (os.path.join('share', package_name, 'webapp'), glob('webapp/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chief-of-mischief',
    maintainer_email='gr2159@nyu.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rosgpt = rosgpt.rosgpt:main',
            'rosgptparser_drone = rosgpt.rosgptparser_drone:main',
            'rosgpt_client_node = rosgpt.rosgpt_client_node:main',
        ],
    },
)
