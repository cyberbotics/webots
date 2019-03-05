#!/bin/bash

echo @@@ Run test suite
echo
python test_suite.py

echo
echo @@@ Run sources untitest
echo
python test_sources.py
