from pathlib import Path

from setuptools import setup  # type: ignore[import]

long_description = (Path(__file__).parent / 'README.md').read_text()

setup(
    name='webots-stubs',
    version='0.1.0',    # TODO: version with the Webots version number?
    url='https://www.cyberbotics.com/',
    project_urls={
        'Issue tracker': 'https://github.com/cyberbotics/webots/issues',
        'Source Code': 'https://github.com/cyberbotics/webots',
    },
    description="Type stubs for Webots' 'controller' package.",
    long_description=long_description,
    long_description_content_type='text/markdown',

    package_data={'controller-stubs': ['*.pyi']},
    packages=['controller-stubs'],

    license='Apache 2.0',

    classifiers=[
        'Development Status :: 2 - Pre-Alpha',
        'Natural Language :: English',
        'Operating System :: OS Independent',
        'Programming Language :: Python',
        'Programming Language :: Python :: 3 :: Only',
        'Programming Language :: Python :: 3.7',
        'Programming Language :: Python :: 3.8',
        'Programming Language :: Python :: 3.9',
        'Programming Language :: Python :: 3.10',
        'Programming Language :: Python :: 3.11',
        'Topic :: Software Development',
    ],
)
