from setuptools import setup

package_name = 'central_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lex watts',
    maintainer_email='lexwatts@u.boisestate.edu',
    description='High-level controller node for navigation and regolith operations.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'central_controller_node = central_controller.central_controller_node:main'
        ],
    },
)
