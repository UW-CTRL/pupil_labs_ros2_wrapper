from setuptools import find_packages, setup

package_name = 'pupil_labs_wrapper'


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    entry_points={
        'console_scripts': [
            'wrapper = pupil_labs_wrapper.wrapper:main',
        ],
    },
    install_requires=['setuptools', 'pupil_labs_realtime_api'],
    
    zip_safe=True,
    maintainer='kysh',
    maintainer_email='kysh@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
)



