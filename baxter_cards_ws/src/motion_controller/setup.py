from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_disutils_setup(
    packages=['common_tools_pkg'],
    package_dir=['':'scripts'],
)

setup(**setup_args)
