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
