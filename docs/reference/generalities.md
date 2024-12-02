## Generalities

Webots world files must use the ".wbt" file name extension.
The first line of a ".wbt" file uses this header:

```
#VRML_SIM V8.5 utf8
```

The version *8.5* specifies that the file can be open with *Webots 8*.
Although the header specifies *utf8*, at the moment only ascii is supported.

The comments placed just below the header store the window configuration associated with this world.

One (and only one) instance of each of the `WorldInfo, ViewPoint` and `Background` nodes must be present in every ".wbt" file.
For example:

```
#VRML_SIM V8.5 utf8

WorldInfo {
  info [
    "Description"
    "Author: first name last name <e-mail>"
    "Date: DD MMM YYYY"
  ]
}
Viewpoint {
  orientation 1 0 0 -0.8
  position 0.25 0.708035 0.894691
}
Background {
  skyColor [
    0.4 0.7 1
  ]
}
PointLight {
  ambientIntensity 0.54
  intensity 0.5
  location 0 1 0
}
```
