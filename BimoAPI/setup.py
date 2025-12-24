from setuptools import setup, find_packages

with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

setup(
    name="mekion-bimo",
    version="0.1.0",
    description="Bimo Robotics Kit - Open-source bipedal robotics platform.",
    long_description=long_description,
    long_description_content_type="text/markdown",
    author="Mekion",
    author_email="info@mekion.com",
    url="https://github.com/mekion/the-bimo-project",
    packages=find_packages(),
    install_requires=[
        "pyserial>=3.5",
        "numpy>=2.2.0",
        "onnxruntime>=1.22.0",
    ],
    python_requires=">=3.8",
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Intended Audience :: Developers",
        "Intended Audience :: Science/Research",
        "License :: OSI Approved :: Apache Software License",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Topic :: Scientific/Engineering :: Reinforcement Learning",
        "Topic :: Scientific/Engineering :: Artificial Intelligence",
        "Topic :: Scientific/Engineering :: Robotics",
    ],
    keywords="robotics biped robot rl reinforcement-learning isaac-lab",
    project_urls={
        "Bug Reports": "https://github.com/mekion/the-bimo-project/issues",
        "Source": "https://github.com/mekion/the-bimo-project",
        "Documentation": "https://www.mekion.com",
    },
)
