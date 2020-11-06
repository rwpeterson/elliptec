import setuptools

with open("README.md", "r") as readme:
    longdesc = readme.read()

setuptools.setup(
    name="elliptec",
    version="0.1.0",
    license="MIT",
    author="Bob Peterson",
    author_email="bob@rwp.is",
    description="Python interface to Thorlabs Elliptec motorized mounts",
    long_description=longdesc,
    long_description_content_type="text/markdown",
    url="https://git.sr.ht/~rwp/elliptec",
    packages=setuptools.find_packages(),
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
        "Topic :: Scientific/Engineering",
        "Topic :: Scientific/Engineering :: Physics",
        "Intended Audience :: Education",
        "Intended Audience :: Science/Research"
    ],
)
