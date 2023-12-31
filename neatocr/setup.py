from setuptools import find_packages, setup

package_name = 'neatocr'

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
    maintainer='cjhi',
    maintainer_email='chilty@olin.edu',
    description='For the project located at https://github.com/cjhi/OCR',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'read_sign = neatocr.read_sign:main',
            'follow_directions = neatocr.follow_directions:main',
            'move_robot = neatocr.move_robot:main'
        ],
    },
)
