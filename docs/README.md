# webots-doc

This repository contains the documentation of the Webots software.

The master branch of this repository is in production on the Cyberbotics website:

- https://www.cyberbotics.com/doc/guide/index
- https://www.cyberbotics.com/doc/reference/index
- https://www.cyberbotics.com/doc/blog/index
- https://www.cyberbotics.com/doc/automobile/index

You are very welcome to contribute to make this documentation better!
In order to proceed, simply fork this repository, make your modifications and
open a pull request that we will review and merge.

## To view a specific version

You can display the documentation corresponding to a specific version of Webots
by using the version argument in the URL, for example:

```
https://www.cyberbotics.com/doc/guide/index?version=8.5
```

This version argument corresponds to a git tag on this repository.

Alternatively, it is possible to display the documentation corresponding to
a github branch of this repository:

```
https://www.cyberbotics.com/doc/guide/index?version=master
```

Or to a github branch (e.g. `reference_proto`) of any public github repository (e.g. `remidhum`):

```
https://www.cyberbotics.com/doc/guide/index?version=remidhum:reference_proto
```

## Run the doc locally

Set the terminal to the 'docs' directory.

```sh
cd $WEBOTS_HOME/docs
```

Create or update the `index.html` page:

```sh
python local_exporter.py
```

Run a simple HTTP server:

```sh
python -m SimpleHTTPServer
```

In a browser, open:

- [http://localhost:8000/?url=&book=guide](http://localhost:8000/?url=&book=guide)
- [http://localhost:8000/?url=&book=reference](http://localhost:8000/?url=&book=reference)
- [http://localhost:8000/?url=&book=automobile](http://localhost:8000/?url=&book=automobile)


## Run the unit tests

Install the pep8 module:

```sh
sudo apt-get install python-pip
pip install pep8
```

Run the tests:

```sh
cd $WEBOTS_HOME/docs
python -m unittest discover
```
