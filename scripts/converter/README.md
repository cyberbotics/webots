# Python scripts to convert Webots world files

The Python scripts located in this folder are used to perform various manipulation on Webots world files ("\*.wbt").
In particular, they are used to convert world files from the R2020a format to R2020b format (with respect to WorldInfo changes).
The webots_parser.py module can be use to open a world file from a Python program, make changes in this world file and save the
resulting file.

These files should be considered as alpha quality: they were not widely tested, not well documented and will change without
providing backwards any compatibility.
They are provided only to help some users to develop scripts for manipulating Webots world files from a Python script.
