from setuptools import find_packages, setup

package_name = "alfred"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/alfred.launch.py"]),
    ],
    install_requires=["setuptools", "openai-whisper", "thefuzz"],
    zip_safe=True,
    maintainer="Ethan Lerner",
    maintainer_email="yaffoman@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "body = alfred.body:main",
            "test_alfred = alfred.test_alfred:main",
            "ear = alfred.ear:main",
            "brain = alfred.brain:main",
        ],
    },
)
