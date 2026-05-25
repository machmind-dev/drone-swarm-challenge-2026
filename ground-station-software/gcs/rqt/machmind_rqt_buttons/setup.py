from setuptools import setup

package_name = "machmind_rqt_buttons"

setup(
    name=package_name,
    version="1.3.5",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "plugin.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Mindaugas",
    maintainer_email="you@example.com",
    description="Mach Mind GCS buttons for rqt",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={},
)
