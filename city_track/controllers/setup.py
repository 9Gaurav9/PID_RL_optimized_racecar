from setuptools import setup, find_packages

setup(
    name='controller_pid_to_rl',
    version='0.1',
    packages=find_packages(),
    install_requires=[
        'gym',
        'numpy',
        'stable-baselines3',
        'matplotlib',       # optional for visualization/logs
    ],
)
