import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="ros_web_gui",
    version="0.0.1",
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
)