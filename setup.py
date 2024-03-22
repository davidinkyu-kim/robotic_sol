from setuptools import find_packages, setup

package_name = 'robotic_sol'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='davkim',
    maintainer_email='gimming9@gmail.com',
    description='Machina Labs homework',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robotic_sol_node = robotic_sol.robotic_sol_node:main'
        ],
    },
)