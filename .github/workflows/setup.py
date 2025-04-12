from setuptools import setup, find_packages

setup(
    name='hqp',
    version='0.1.0',
    author='Vladlen',
    author_email='vladlen.codes@gmail.com',
    description='A pathfinding algorithm using hierarchical quantum principles for navigating mazes with dynamic obstacles.',
    long_description=open('README.md').read(),
    long_description_content_type='text/markdown',
    url='https://github.com/vladlen-codes/HQP',
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