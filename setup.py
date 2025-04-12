from setuptools import setup, find_packages

setup(
    name='hierarchical-quantum-pathfinding',
    version='0.1.0',
    author='Your Name',
    author_email='vladlen.codes@gmail.com',
    description='A pathfinding algorithm using hierarchical quantum principles for navigating mazes with dynamic obstacles.',
    long_description=open('README.md').read(),
    long_description_content_type='text/markdown',
    url='https://github.com/yourusername/hierarchical-quantum-pathfinding',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    classifiers=[
        'Programming Language :: Python :: 3',
        'License :: OSI Approved :: MIT License',
        'Operating System :: OS Independent',
    ],
    python_requires='>=3.6',
    install_requires=[
        'numpy',
    ],
    include_package_data=True,
)