### Usage

```
cd tests

# run the entire test suite
./test_suite.py

# don't perform the initial make stage
./test_suite.py --nomake

# run tests individually
./test_suite.py api/worlds/gps.wbt parser/worlds/empty_value.wbt

# run tests individually without the test suite framework (the test_suite_supervisor returns directly)
../webots api/worlds/gps.wbt
```

### Missing tests

- gyro
- propeller
- led
- position sensors
- charger
- physics friction test
- physics damping test
- joints

### WREN camera cache regression (Linux)

This CPU test builds from WREN sources and needs a C++11 compiler, GNU ld, and the
pinned GLM submodule. It runs in the Linux Test Sources CI job without Webots or a
GPU context.

```sh
git submodule update --init src/glm
make -C tests/wren test
```
