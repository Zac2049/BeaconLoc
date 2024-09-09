from setuptools import setup, find_packages

setup(
    name='beaconloc',
    version='0.1',
    packages=find_packages(),
    install_requires=[
        'numpy',
        'scipy',
        'matplotlib',
        'pandas',
        'pyyaml'
    ],
    author='Li Penghui',
    author_email='perfectblue@duck.com',
    description='A package for beacon localization',
    long_description=open('README.md').read(),
    long_description_content_type='text/markdown',
    url='https://github.com/Zac2049/BeaconLoc',
    package_dir={'': 'beaconloc'},
    include_package_data=True,
    classifiers=[
        'Development Status :: 3 - Alpha',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache 2.0',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.7',
        'Programming Language :: Python :: 3.8',
        'Programming Language :: Python :: 3.9',
        'Programming Language :: Python :: 3.10',
        'Programming Language :: Python :: 3.11',
        'Programming Language :: Python :: 3.12'
    ],
    python_requires='>=3.7'
)
