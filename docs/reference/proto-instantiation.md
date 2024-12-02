## PROTO Instantiation

Each PROTO instance can be considered to be a complete copy of the PROTO, with its interface fields and body nodes.
PROTO are instantiated using the standard node syntax, for example:

```
Bicycle {
  position   0 0.5 0
  frameColor 0 0.8 0.8
  hasBrakes  FALSE
}
```

When PROTO instances are read from a ".wbt" file, field values for the fields of the PROTO interface may be given.
If given, the field values are used for all nodes in the PROTO definition that have IS statements for those fields.
