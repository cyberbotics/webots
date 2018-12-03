## A Typical OpenstreetMap Importer Configuration File

```ini
# BUILDINGS DEFAULTS SETTINGS #

[building]
ignore: FALSE
floorHeight: 3
floorNumber: 3

[building_house]
ignore: FALSE
floorHeight: 3
floorNumber: 2

[building_apartments]
ignore: FALSE
floorHeight: 3
floorNumber: 4

[building_residential]
ignore: FALSE
floorHeight: 3.2
floorNumber: 6

# WATERWAY DEFAULTS SETTINGS #

[waterway]
ignore: TRUE

[waterway_river]
ignore: FALSE
width: 2

[waterway_stream]
ignore: FALSE
width: 2

# AREA DEFAULTS SETTINGS #

[area]
ignore: TRUE

[area_farmland]
ignore: FALSE
texture: textures/dry_grass.jpg

[area_farm]
ignore: FALSE
texture: textures/dry_grass.jpg

[area_water]
ignore: FALSE
blueComponent: 1

[area_forest]
ignore: FALSE
greenComponent: 1
transparency: 0.7
density: 0.1

# ROAD DEFAULTS SETTINGS #

[road]
ignore: TRUE
filter: 19780138, 20561790, 339403904, 19779339, 199529802, 339403894, 20560490, 19778494, 20561964, 483687553, 142531625, 19778614, 199523325, 339412260, 339403907, 339403908  # list of roads used to filter-out unimportant objects

[road_motorway]
ignore: FALSE
border: FALSE
# this is for oneway
laneNumber: 3
laneWidth: 3.5

[road_primary]
ignore: FALSE
border: FALSE
laneNumber: 2
laneWidth: 3.5

[road_primary_link]
ignore: FALSE
laneNumber: 1
laneWidth: 4.5

[road_secondary]
ignore: FALSE
laneNumber: 2
laneWidth: 3.5
```
