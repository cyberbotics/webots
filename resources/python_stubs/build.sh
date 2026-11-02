#!/bin/sh

cd $(dirname $0)

rm -rf controller-stubs

stubgen ../../lib/controller/python/controller --output .

mv controller controller-stubs

python setup.py sdist bdist_wheel
