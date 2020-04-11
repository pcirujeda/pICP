# pICP
This project provides a clean and fast implementation of the ICP method by [Besl and McKay](https://ieeexplore.ieee.org/document/121791/) in C++ with minimal dependencies (see below). The algorithm is one of the baseline methods on rigid alignment of point clouds, and this implementation attempts to provide an entry point for learning it with no practical hassles. At the same time, it works fast and robustly enough to be used out-of-the box on any of your projects.

![](docs/alignment.png)


## How it works
### The problem
Given two point sets:

![](docs/readme_eq1.gif)

![](docs/readme_eq2.gif)

find a rotation R and translation T that minimizes the error:

![](docs/readme_eq3.gif)

### SVD based transform estimation
Given a matrix W made by a set of correspondent N points, centered to its mean

![](docs/readme_eq4.gif)

obtain the Singular Value Decomposition of W = U * S * Vt.

Theorem without proof says that if rank( W ) = 3, the optimal solution of E( R, t ) is unique and given by:

![](docs/readme_eq5.gif)

![](docs/readme_eq6.gif)

This process can be run iteratively, selecting correspondent points by different criteria (neighbourhood, random search...), until convergence.

## Dependencies
This project attempts to use the minimal dependencies as possible, and those are self-contained in the project when possible.

* [Nanoflann](https://github.com/jlblancoc/nanoflann) for KDTree-based correspondences search. Self-contained.
* [TinyObjLoader](https://github.com/syoyo/tinyobjloader) for OBJ file parsing. Self-contained.
* [googletest](https://github.com/google/googletest) for Unit testing. Self-contained (downloads and installs automatically).
* [Eigen](http://eigen.tuxfamily.org) for matrix manipulation and SVD computation. Required on your system.
* [Boost](https://www.boost.org/) just for parsing command-line arguments into the PlainICP executable. Required on your system.

## Usage
Make sure Eigen and Boost are installed on your system and compile the full project:

```
sudo brew install eigen boost                         # For OsX
sudo apt-get install libeigen3-dev libboost-all-dev   # For Linux apt based distros

mkdir ./build
cd ./build
cmake ..
make
```

Sample call with provided sample OBJ files:

```
./PlainICP --source-obj-file ../test/data/bunny_head.obj              \
           --target-obj-file ../test/data/bunny.obj                   \
           --transformed-obj-file ../test/data/aligned_bunny_head.obj \
           --tolerance 0.005                                          \
           --verbose 1
```

If you want to integrate the ICP implementation into your project, just copy the headers into your source code.

## Testing
Unit testing is provided. Run the following target in the build directory:

```
make build_and_test
```

## License
This software release is primarily [MIT](https://opensource.org/licenses/MIT) licensed. Some files contain third-party code under other license.