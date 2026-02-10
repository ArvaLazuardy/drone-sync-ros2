from setuptools import find_packages, setup

package_name = 'mata_mata'

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
    maintainer='heeving',
    maintainer_email='heeving@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'talker = mata_mata.publisher_member_function:main',
            'listener = mata_mata.subscriber_member_function:main', 
            'service = mata_mata.service_member_function:main',
            'client = mata_mata.client_member_function:main',
        ],
    },
)
