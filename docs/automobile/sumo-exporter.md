# SUMO Exporter

In order to be able to simulate traffic in your simulation, it is required to have the SUMO package installed on the system and a SUMO network file (`sumo.net.xml`).
The SUMO exporter can create SUMO network files from a Webots simulation.

## Dependencies

Please refer to [these instructions](sumo-interface.md#dependencies) to install SUMO.

The SUMO exporter is also using the `shapely` Python module.
Please refer to [these instructions](openstreetmap-importer.md#dependencies) to install it.

## Expectations on the Webots Simulation

If the Webots simulation has been created from the OpenStreetMap importer, then the export should be straight forward.

If it's not the case, you should pay attention on the use of the [Road](https://webots.cloud/run?url={{ url.github_tree }}/projects/objects/road/protos/Road.proto) and the [Crossroad](https://webots.cloud/run?url={{ url.github_tree }}/projects/objects/road/protos/Crossroad.proto) PROTO nodes.
Indeed, their IDs should be unique, and the `Road.startJunction`, the `Road.endJunction` and the `Crossroad.connectedRoadIDs` fields should be filled correctly.

## How to use the Exporter

You can use the `exporter.py` Python script to generate the `sumo.nod.xml`, `sumo.edg.xml` and `sumo.sumocfg` SUMO files.
These files can be used by SUMO `netconvert` to generate the `sumo.net.xml` file from the `myMap.wbt` webots simulation world.

%tab-component "os"

%tab "Linux"

```sh
cd $WEBOTS_HOME/resources/sumo_exporter
mkdir myMap_net
python exporter.py --input=myMap.wbt --output=myMap_net
$SUMO_HOME/bin/netconvert --node-files=myMap_net/sumo.nod.xml --edge-files=myMap_net/sumo.edg.xml --output-file=myMap_net/sumo.net.xml
```

%tab-end

%tab "macOS"

```sh
cd $WEBOTS_HOME/Contents/Resources/sumo_exporter
mkdir myMap_net
python exporter.py --input=myMap.wbt --output=myMap_net
$SUMO_HOME/bin/netconvert --node-files=myMap_net/sumo.nod.xml --edge-files=myMap_net/sumo.edg.xml --output-file=myMap_net/sumo.net.xml
```

%tab-end

%end

The resulting `sumo.net.xml` file can be open in SUMO `netedit` for some edition.
For example the connections between junctions can be improved at this stage.

Note that SUMO `netedit` can display weird widths and offsets for the lanes during the first load of the network file.
This can be solved simply by saving the unmodified `sumo.net.xml` file or by selecting the `Processing / Compute Junctions` menu item.

## Arguments

| Argument   | Description                                                       | Default value |
| ---------- | ----------------------------------------------------------------- | ------------- |
| `--input`  | Specifies the Webots .wbt file to open.                           | "file.wbt"    |
| `--output` | Specifies the directory where to generate the SUMO network files. | "."           |
