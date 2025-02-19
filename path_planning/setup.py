from setuptools import find_packages, setup

package_name = "path_planning"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/resource", ["resource/workspace_1.tsv"]),
        ("share/" + package_name + "/resource", ["resource/workspace_2.tsv"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="robot",
    maintainer_email="jule",
    description="TODO: Package description",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "point_publisher = path_planning.point_publisher:main",
            "random_point = path_planning.random_point:main",
            "visualize_ws = path_planning.visualize_ws:main",
        ],
    },
)
