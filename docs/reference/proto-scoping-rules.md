## PROTO Scoping Rules

PROTO names must be unique: defining a PROTO with the same name as another PROTO or a built-in node type is an error.
A ".proto" file can contain only one PROTO definition.
A PROTO node can be defined in terms of other PROTO nodes.
However, instantiation of a PROTO inside its own definition is not permitted (i.e., recursive PROTO are illegal).
An IS statement refers to a field in the interface of the same PROTO, in the same file.
Fields declared in the interface can be passed to sub-PROTO nodes using IS statements.

A ".proto" file establishes a DEF/USE name scope separate from the rest of the scene tree and separate from any other PROTO definition.
Nodes given a name by a DEF construct inside the PROTO may not be referenced in a USE construct outside of the PROTO's scope.
Nodes given a name by a DEF construct outside the PROTO scope may not be referenced in a USE construct inside the PROTO scope.

In case of derived PROTO nodes, it is allowed to declare in the interface a field with the same name as a base PROTO field only if it is not associated with any other field than the homonymous base PROTO field.
This means that it is possible to use the derived field in template statements without restrictions, but if it used in a IS statement then the two identifiers before and after the IS keyword have to match.
If the derived field has a unique name then no restrictions apply.
