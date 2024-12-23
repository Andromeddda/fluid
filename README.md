# fluid

2nd and 3rd homework for HSE SE C++ course, 3nd year.

## Report

In file ./REPORT.md

## Requirements

* C++20 compiler
* CMake version at least 3.23

## Cloning into repository
```
git clone https://github.com/Andromeddda/fluid
```
```
cd fluid
```

## Building

### Fast building (bash shell)
```
./install
```

### Manual building
```
cmake -DCMAKE_BUILD_TYPE="RELEASE" -B build -S . -DTYPES=... [-DSIZES=...]
```
```
cmake --build build --target fluid -j 4
```

## Launching
* The order of command line options is arbitrary
```
./build/fluid --p-type='...' --v-type='...' --vf-type='...' filename '--save-to=file' [-jN]
```
* The project also has two bash scripts that build and launch prepared examples
```
./launch_example_1
```
```
./launch_example_2
```
```
./launch_example_j
```

## Saving to file
### fluid-2 (branch develop)

* To save current field position to file, type newline character (``'\n'`` or ASCII 10) by pressing [ENTER]

### fluid-3 (branch fluid-3)

* This version does not maintain saving to file. Command line option '--save-to=...' will still parse.
