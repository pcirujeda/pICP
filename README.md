# pICP
This repo provides a clean and fast implementation of the ICP method by [Besl and McKay](https://ieeexplore.ieee.org/document/121791/) in C++ with minimal dependencies (in fact, only [OpenCV](https://opencv.org/) is strictly required for data matrix manipulation and SVD computation). The algorithm is one of the baseline methods on rigid alignment of point clouds, and this implementation attempts to provide an entry point for learning it with no practical hassles. At the same time, it works fast an robustly enough to be used out-of-the box on any of your projects.

## How it works
### The problem
Given two point sets:

![](https://latex.codecogs.com/gif.download?Source%20%3D%20%5Cleft%20%5C%7B%20source_1%2C%20...%2C%20source_N%20%5Cright%20%5C%7D)
![](https://latex.codecogs.com/gif.download?Target%20%3D%20%5Cleft%20%5C%7B%20target_1%2C%20...%2C%20target_N%20%5Cright%20%5C%7D)

find a rotation R and translation T that minimizes the error:
![](https://latex.codecogs.com/gif.download?E%28R%2C%20t%29%20%3D%20%5Cfrac%7B1%7D%7BN%7D%5Csum_%7Bi%3D1%7D%5E%7BN%7D%5Cleft%20%5C%7C%20target_i%20-%20R%20%5Ccdot%20source_i%20-%20t%20%5Cright%20%5C%7C%5E2)

### SVD based transform estimation
Given a matrix W made by a set of correspondent N points, centered to its mean
![](https://latex.codecogs.com/gif.download?W%20%3D%20%5Csum_%7Bi%3D1%7D%5E%7BN%7D%20%28target_i%20-%20%5Cmu_%7BTarget%7D%29%20%5Ccdot%20%28source_i%20-%20%5Cmu_%7BSource%7D%29)

obtain the Singular Value Decomposition of W = U * S * Vt.

Theorem without proof says that if rank( W ) = 3, the optimal solution of E( R, t ) is unique and given by:
![](https://latex.codecogs.com/gif.download?R%20%3D%20U%20V%5ET)
![](https://latex.codecogs.com/gif.download?t%20%3D%20%5Cmu_%7B%20Target%20%7D%20-%20R%20%5Ccdot%20%5Cmu_%7BSource%7D)

This process can be run iteratively, selecting correspondent points by different criteria (neighbourhood, random search...), until convergence.

## Dependencies
Only OpenCV is required if you are just using the ICP class on your project.

A real test executable is provided, allowing to register any desired pair of point clouds (only .obj format supported at the moment). The PointCloud class uses [TinyObjLoader](https://github.com/syoyo/tinyobjloader) (header included) for parsing OBJ files, and the PlainICP executable requires [Boost](https://www.boost.org/) for parsing external arguments.

## Usage
Just include the ICP class header on your project. Check the provided cmake project for compiling an executable which shows how to use the class:

```
mkdir ./build
cd ./build
cmake ..
make

./PlainICP --h
```

## License
This software release is primarily [MIT](https://opensource.org/licenses/MIT) licensed. Some files contain third-party code under other licens