import re
import setuptools

with open("ros_web_gui/__init__.py", encoding="utf8") as f:
    version = re.search(r'__version__ = "(.*?)"', f.read()).group(1)

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="ros_web_gui",
    version=version,
    author="Simon Manschitz",
    author_email="simon.manschitz@gmx.de",
    description="A web gui for ROS (robot operating system)",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/smanschi/ros_web_gui",
    packages=setuptools.find_packages(),
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires='>=3.6',
    scripts=['scripts/ros_web_gui'],
    package_data={'ros_web_gui': ['*', 'static/*', 'templates/*']},
    include_package_data=True,
    install_requires=[
        'Flask>=1.1.2',
        'PyYAML>=5.3.1',
        'pygraphviz>=1.3'
    ]
)
