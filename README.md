# PCL Normals

"Simple" code to estimate normals from point clouds using the PCL library.

# Compiling

Needs CMake and PCL.

```
cd /path/to/repo
mkdir build
cd build
cmake ..
```

# Running
Two variables, `plyFilename` and `searchRadius`, can be changed within the code. Otherwise, you can pass both of these values as command line arguments.

Usage:
```
./main plyFilename searchRadius
```
It can also run without any arguments, but the values should be changed within the code and recompiled.

Example:
```
./main /path/to/ply 100
```
Will run the code on the `.ply` file specified with a 100 unit sphere. The radius itself should be changed depending on the data.
