# pICP
This repo provides a clean and fast implementation of the ICP method by [Besl and McKay](https://ieeexplore.ieee.org/document/121791/) in C++ with minimal dependencies (in fact, only [OpenCV](https://opencv.org/) is strictly required for data matrix manipulation and SVD computation). The algorithm is one of the baseline methods on rigid alignment of point clouds, and this implementation is intended either for an entry point for learning it, as well as to be used out-of-the box on any project.

## How it works
TODO

## Dependencies
Only OpenCV is required if you are just using the ICP class on your project.

A real test executable is provided, allowing to register any desired pair of point clouds (only .obj format supported at the moment). The PointCloud class uses [TinyObjLoader](https://github.com/syoyo/tinyobjloader) (header included) for parsing OBJ files, and [Boost](https://www.boost.org/) for parsing external arguments.

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
This software release is primarily [MIT](https://opensource.org/licenses/MIT) licensed. Some files contain third-party code under other licenses.