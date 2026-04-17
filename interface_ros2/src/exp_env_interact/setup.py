from setuptools import find_packages, setup

package_name = 'exp_env_interact'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/docs', [
            'docs/pitfalls.md',
            'docs/usage.md',
        ]),
    ],
    install_requires=['setuptools', 'numpy', 'scipy'],
    zip_safe=True,
    maintainer='maintainer',
    maintainer_email='user@example.com',
    description='ROS2 Virtuose expert + Franka / RViz2 env.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'exp_env_interact_node = exp_env_interact.exp_env_interact_node:main',
        ],
    },
)
