from setuptools import setup

package_name = 'mpc'

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
    maintainer='sathish gopalakrishnan, farshid agharebparast',
    maintainer_email='sathish@ece.ubc.ca, farshid@ece.ubc.ca',
    description='follow_the_gap lab',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'reactive_node = gap_follow.reactive_node:main',
        ],
    },
)
