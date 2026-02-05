from setuptools import setup
from setuptools import Extension

setup(
    name='OpenEAI-Arm',
    version='0.1',
    packages=['OpenEAIArm'],
    package_dir={'OpenEAIArm': 'OpenEAIArm'},
    package_data={'OpenEAIArm': ['OpenEAIArm*.so']},
    include_package_data=True,
)