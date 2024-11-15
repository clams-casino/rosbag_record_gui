from setuptools import setup, find_packages

setup(
    name='rosbag_record_gui',
    version='1.0.0',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    install_requires=[
        'rospy',
        'PyQt5',
        'numpy',
        'matplotlib',
    ],
)