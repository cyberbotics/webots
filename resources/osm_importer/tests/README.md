# OSM elevation regression tests

Install the importer dependencies (`pyproj`, `shapely`, `lxml`, and `webcolors`) and run from the repository root:

```sh
python -m unittest discover -s resources/osm_importer/tests -v
```

These tests use the real projection, importer, and world exporters with a synthetic OSM file.
Only the remote elevation service is replaced with a deterministic slope. No service credentials
or simulator installation are required; they check generated world data, not rendered output.
