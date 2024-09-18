from setuptools import find_packages, setup

package_name = 'my_tf2_package'

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
    maintainer='yyj',
    maintainer_email='yyj@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_static_cart_tf2_broadcaster = my_tf2_package.my_static_cart_tf2_broadcaster:main'
        ],
    },
)
