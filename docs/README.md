# Webots Documentation

This repository contains the documentation for the Webots open source simulation software.

The released branch of this repository is in production on the Cyberbotics website, which you can access by visiting the following links:

- https://www.cyberbotics.com/doc/guide/index
- https://www.cyberbotics.com/doc/reference/index
- https://www.cyberbotics.com/doc/blog/index
- https://www.cyberbotics.com/doc/discord/index
- https://www.cyberbotics.com/doc/automobile/index

You are very welcome to contribute to make this documentation better.
In order to proceed, follow these steps:
1. Fork this repository
2. Make your modifications
3. Open a pull request that we will review and merge.

## To view a specific version:

You can display the documentation corresponding to a specific version of Webots
by using the version argument in the URL, for example:

```
https://www.cyberbotics.com/doc/guide/index?version=8.5
```

This version argument corresponds to a git tag on this repository.

Alternatively, it is possible to display the documentation corresponding to
a github branch of this repository:

```
https://www.cyberbotics.com/doc/guide/index?version=develop
```

Or to a github branch (e.g. `reference_proto`) of any public github repository (e.g. `remidhum`):

```
https://www.cyberbotics.com/doc/guide/index?version=remidhum:reference_proto
```

## To Run the doc locally, follow these steps:

1. Set the terminal to the 'docs' directory.

```sh
cd $WEBOTS_HOME/docs
```

2. Create or update the `index.html` page:

```sh
python local_exporter.py
```

3. Run a simple HTTP server:

```sh
python -m http.server 8000
```

4. Then in a browser, open:

- [http://localhost:8000/?url=&book=guide](http://localhost:8000/?url=&book=guide)
- [http://localhost:8000/?url=&book=reference](http://localhost:8000/?url=&book=reference)
- [http://localhost:8000/?url=&book=blog](http://localhost:8000/?url=&book=blog)
- [http://localhost:8000/?url=&book=automobile](http://localhost:8000/?url=&book=automobile)


## For running the unit tests, the following steps should be followed:

1. Install the pycodestyle module:

```sh
sudo apt-get install python-pip
pip install pycodestyle
```

2. Run the tests:

```sh
cd $WEBOTS_HOME/docs
python -m unittest discover
```
