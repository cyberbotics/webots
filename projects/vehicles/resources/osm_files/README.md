## Content of the folder
This folder contains OSM files (part of OSM world map) for some specific locations. Those files have been exctracted from OSM and the edited in order to rendered nicely when converted to a Webots world file using the script.

## Editing these files
To edit/display these file use [JOSM](https://josm.openstreetmap.de) it is a very powerful, easy to use and cross-platform software.
To remove deleted item (e.g. node) definitely, use the following command:
```
xmlstarlet ed -d "/osm/*[@action='delete']" < input.osm > output.osm
```

## Get new files
To get new file you can easily extract them from OpenStreetMap using [OSM export](https://www.openstreetmap.org/export)

## Files description
* CH_Morges: this file represent the city of Morges in Switzerland (no 3D)
* CH_Vens: this file represent a mountain road and a small village in the Alpes to be rendered in 3D
* village: small village created from scratch to be used for high level of details simulation
