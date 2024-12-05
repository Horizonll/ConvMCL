from setuptools import find_packages, setup

package_name = "convmcl"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="hrz",
    maintainer_email="herz23@mails.tsinghua.edu.cn",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "recorder = convmcl.recorder:main",
            "nav_through_poses = convmcl.nav_through_poses:main",
            "nav_recorder = convmcl.nav_recorder:main",
            "init = convmcl.init:main",
        ],
    },
)
