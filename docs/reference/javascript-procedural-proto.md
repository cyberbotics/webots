## Procedural PROTO Nodes

Procedural PROTO nodes can be created using JavaScript as a scripting language.
Introducing and learning JavaScript is outside the scope of this document.

### Template Engine

In the event that a PROTO file contains JavaScript template statements, as denoted by "%{" and the "}%" tokens, a template engine is used to evaluate it.
The contents of the file are first translated to pure JavaScript that is successively evaluated using QJSEngine, yielding a VRML97 compatible PROTO file which is then parsed by Webots as any other PROTO would.

### Programming Facts

- A template statement is encapsulated inside the "%{" and the "}%" tokens and can be written on several lines.
- Adding an "=" just after the opening token ("%{=") allows to evaluate a statement.
- The use of template statements is exclusively allowed inside the content scope of the PROTO (cf. example).
- The fields are accessible in a globally accessible object named "fields".
The keys of this object matches the PROTO's field names and for each entry, an additional sub-object with two keys named "value" and "defaultValue" is available.
The first one contains the current value of the field and the second one contains its default state.
The conversion between the VRML97 types and the JavaScript types is detailed in [this table](#vrml97-type-to-javascript-type-conversion).
- As shown in [this table](#vrml97-type-to-javascript-type-conversion), the conversion of a VRML97 node is an object.
This object contains the following keys: "node\_name" containing the VRML97 node name and "fields" which is in turn an object containing the JavaScript representation of the VRML97 node fields.
This object is equal to `undefined` if the VRML97 node is not defined (`NULL`).
- The `context` object provides contextual information about the PROTO.
Table [this table](#content-of-the-context-object) shows the available information and the corresponding keys.
- The VRML97 comment ("#") prevails over the JavaScript statements.
- JavaScript standard output and error streams are redirected on the Webots console (written respectively in regular and in red colors).
This allows developers to use `console.log` and `console.error` to write on these streams.

#### VRML97 Type Conversion

%figure "VRML97 type to JavaScript type conversion"

| VRML97 type  | JavaScript type                                       | Example                     |
| ------------ | ----------------------------------------------------- | --------------------------- |
| SFBool       | boolean                                               | true                        |
| SFInt32      | number                                                | 10                          |
| SFFloat      | number                                                | 10.0                        |
| SFString     | string                                                | "Webots"                    |
| SFVec2f      | object (keys: "x" and "y")                            | {x: 1, y: 2}                |
| SFVec3f      | object (keys: "x", "y" and "z")                       | {x: 1, y: 2, z: 3}          |
| SFRotation   | object (keys: "x", "y", "z" and "a")                  | {x: 1, y: 2, z: 3, a: 3.14} |
| SFColor      | object (keys: "r", "g" and "b")                       | {r: 1, g: 0, b: 0}          |
| SFNode       | object (keys: "node\_name", "fields")                 |                             |
| MF*          | array (indexes = multiple value positions)            |                             |

%end

%figure "Content of the context object"

| Key                     | Value                                                                                                                                                                                                                                     |
| ----------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| world                   | absolute path to the current world file (including file name and extension)                                                                                                                                                               |
| proto                   | absolute path to the current PROTO file (including file name and extension)                                                                                                                                                               |
| project\_path           | absolute path to the current project directory                                                                                                                                                                                            |
| webots\_version         | object representing the version of Webots with which the PROTO is currently used (keys: "major" and "revision")                                                                                                                |
| webots\_home            | absolute path to the Webots installation directory                                                                                                                                                                                        |
| temporary\_files\_path  | absolute path to the temporary folder currently used by Webots (this is the location where the PROTO file is generated)                                                                                                                   |
| os                      | OS string ("windows", "linux" or "mac")                                                                                                                                                                                                   |
| id                      | id of the node. This id is equivalent to the one returned by the [wb\_supervisor\_node\_get\_id](supervisor.md#wb_supervisor_node_get_from_def) function and may be used for example to initialize the seed of a random number generator. |
| coordinate_system       | value of the [WorldInfo](worldinfo.md).`coordinateSystem` field.                                                                                                                                                                          |

%end

#### Utility Modules

A number of modules provide additional utility functions that can be useful when creating procedural PROTO files.
To use these functions, the module needs to be included first:

```
%{
  // to import the entire module
  import * as wbrotation from 'modules/webots/wbrotation.js';

  // access functions
  let number = wbrandom.integer(0, 10);

  // to import only specific functions instead of the entire module
  import {integer} from 'modules/webots/wbrotation.js';
}%
```

The available modules are the following.

- `wbrandom`: provides functions that allow seed-based pseudo-random number generation.

- `wbrotation`: provides utility functions for dealing with rotations, represented in axis-angle, quaternions or matrix format.

- `wbvector2`: provides utility functions to manipulate two-dimensional vectors.

- `wbvector3`: provides utility functions to manipulate three-dimensional vectors.

- `wbmath`: provides commonly needed mathematical functions.

- `wbgeometry`: provides geometry related functions, allowing for instance to create splines.

The functions exported by each module are:

%tab-component "module"

%tab "wbrandom"
```
/**
 * @param {number} s
 */
wbrandom.seed(s);
```

Sets the random number generator seed to the specified value.
The numbers are generated using a Linear Congruential Generator (LCG) algorithm.
To generate non-deterministic results, a time-based seed can be used `wbrandom.seed(Date.now())`.

```
/**
 * @param {number} [min]
 * @param {number} [max]
 * @returns {number}
 */
wbrandom.real(min, max);
```

Returns a floating point pseudo-random number in the range [min, max].
If only one parameter is provided, the number will be between zero and that value.
If no parameters are provided, it returns a value in the range [0, 1].

```
/**
 * @param {number} [min]
 * @param {number} [max]
 * @returns {number}
 */
wbrandom.integer(min, max);
```

Returns a pseudo-random integer number in the range [min, max].
If only one parameter is provided, the number will be between zero and it.
If no parameters are provided, it returns a value in the range [0, 2^24 - 1].

%tab-end

%tab "wbrotation"
```
/**
 * @param {Object.<x: number, y: number, z: number, a: number>} rA
 * @param {Object.<x: number, y: number, z: number, a: number>} rB
 * @returns {boolean}
 */
wbrotation.equal(rA, rB);
```

Returns true if the two rotation vectors, in axis-angle format, are equal.
Returns false otherwise.

```
/**
 * @param {Object.<w: number, x: number, y: number, z: number>} q
 * @returns {Object.<x: number, y: number, z: number, a: number>}
 */
wbrotation.fromQuaternion(q);
```

Converts a rotation provided as quaternion to an axis-angle representation.

```
/**
 * @param {Object.<x: number, y: number, z: number, a: number>} r
 * @returns {Object.<w: number, x: number, y: number, z: number>}
 */
wbrotation.toQuaternion(r);
```

Converts a rotation in axis-angle representation to quaternion format.

```
/**
 * @param {Object.<0: number, 1: number, ..., 8: number>} m
 * @returns {Object.<x: number, y: number, z: number, a: number>}
 */
wbrotation.fromMatrix3(m);
```

Converts a rotation in matrix form (3 x 3) to an axis-angle representation.

```
/**
 * @param {Object.<x: number, y: number, z: number, a: number>} r
 * @returns {Object.<0: number, 1: number, ..., 8: number>}
 */
wbrotation.toMatrix3(r);
```

Converts a rotation in axis-angle representation to matrix (3 x 3) format.

```
/**
 * @param {Object.<x: number, y: number, z: number, a: number>} r
 * @returns {boolean}
 */
wbrotation.isIdentity(r);
```

Returns true if the rotation provided in axis-angle representation corresponds to the identity matrix.

```
/**
 * @param {Object.<x: number, y: number, z: number, a: number>} r
 * @returns {Object.<x: number, y: number, z: number, a: number>}
 */
wbrotation.normalize(r);
```

Normalizes both the axis and the angle of the provided rotation vector.

```
/**
 * @param {Object.<x: number, y: number, z: number, a: number>} rA
 * @param {Object.<x: number, y: number, z: number, a: number>} rB
 * @returns {Object.<x: number, y: number, z: number, a: number>}
 */
wbrotation.combine(rA, rB);
```

Combines two rotations, in axis-angle format, together.

```
/**
 * @param {Object.<0: number, 1: number, ..., 8: number>} m
 * @param {Object.<x: number, y: number, z: number>} v
 * @returns {Object.<x: number, y: number, z: number>}
 */
wbrotation.rotateVector3ByMatrix3(m, v);
```

Rotates the 3-dimensional vector `v` according to the rotation matrix `m`.

```
/**
 * @param {Object.<x: number, y: number, z: number, a: number>} r
 * @param {Object.<x: number, y: number, z: number>} v
 * @returns {Object.<x: number, y: number, z: number>}
 */
wbrotation.rotateVector3ByRotation(r, v);
```

Rotates the 3-dimensional vector `v` according to the rotation vector `r`.

```
/**
 * @param {Object.<w: number, x: number, y: number, z: number>} q
 * @param {Object.<x: number, y: number, z: number>} v
 * @returns {Object.<x: number, y: number, z: number>}
 */
wbrotation.rotateVector3ByQuaternion(q, v);
```

Rotates the 3-dimensional vector `v` according to the rotation  `q`.

%tab-end

%tab "wbvector2"
```
/**
 * @param {Object.<x: number, y: number>} vA
 * @param {Object.<x: number, y: number>} vB
 * @returns {boolean}
 */
wbvector2.equal(vA, vB);
```

Returns true if the two vectors are equal.

```
/**
 * @param {Object.<x: number, y: number>} vA
 * @param {Object.<x: number, y: number>} vB
 * @returns {Object.<x: number, y: number>}
 */
wbvector2.add(vA, vB);
```

Returns the vectorial sum of the two vectors.

```
/**
 * @param {Object.<x: number, y: number>} vA
 * @param {Object.<x: number, y: number>} vB
 * @returns {Object.<x: number, y: number>}
 */
wbvector2.minus(vA, vB);
```

Returns the vectorial difference of the two vectors.

```
/**
 * @param {Object.<x: number, y: number>} v
 * @param {number} s
 * @returns {Object.<x: number, y: number>}
 */
wbvector2.multiply(v, s);
```

Multiplies the vector `v` by the scalar `s` and returns the result.

```
/**
 * @param {Object.<x: number, y: number>} v
 * @returns {number}
 */
wbvector2.norm(v);
```

Returns the norm of the provided vector.

```
/**
 * @param {Object.<x: number, y: number>} v
 * @returns {number}
 */
wbvector2.atan2(v);
```

Returns the angle in the plane (in radians) between the positive x-axis and the ray from (0, 0) to the point `v`.
It should be noted that what this function returns is `Math.atan2(v.y, v.x)`.

```
/**
 * @param {Object.<x: number, y: number>} vA
 * @param {Object.<x: number, y: number>} vB
 * @returns {number}
 */
wbvector2.distance(vA, vB);
```

Returns the shortest distance between the two vectors.

```
/**
 * @param {Object.<x: number, y: number>} vA
 * @param {Object.<x: number, y: number>} vB
 * @returns {number}
 */
wbvector2.angle(vA, vB);
```

Returns the angle between the two vectors.

```
/**
 * @param {Object.<x: number, y: number>} vA
 * @param {Object.<x: number, y: number>} vB
 * @returns {number}
 */
wbvector2.cross(vA, vB);
```

Returns the cross product between the two vectors.

```
/**
 * @param {Object.<x: number, y: number>} vA
 * @param {Object.<x: number, y: number>} vB
 * @returns {number}
 */
wbvector2.dot(vA, vB);
```

Returns the dot product between the two vectors.

```
/**
 * @param {Object.<x: number, y: number>} v
 * @returns {Object.<x: number, y: number>}
 */
wbvector2.normalize(v);
```

Normalizes the provided vector.

```
/**
 * @param {Object.<x: number, y: number>} p1
 * @param {Object.<x: number, y: number>} p2
 * @param {Object.<x: number, y: number>} p3
 * @param {Object.<x: number, y: number>} p4
 * @returns {Object.<x: number, y: number>}
 */
wbvector2.intersection(p1, p2, p3, p4);
```

Returns the intersecting point between the segments [p1, p2] and [p3, p4].
Returns null if they do not intersect.

%tab-end

%tab "wbvector3"
```
/**
 * @param {Object.<x: number, y: number, z: number>} vA
 * @param {Object.<x: number, y: number, z: number>} vB
 * @returns {boolean}
 */
wbvector3.equal(vA, vB);
```

Returns true if the two 3-dimensional vectors are equal.

```
/**
 * @param {Object.<x: number, y: number, z: number>} vA
 * @param {Object.<x: number, y: number, z: number>} vB
 * @returns {Object.<x: number, y: number, z: number>}
 */
wbvector3.add(vA, vB);
```

Returns the vectorial sum of the two vectors.

```
/**
 * @param {Object.<x: number, y: number, z: number>} vA
 * @param {Object.<x: number, y: number, z: number>} vB
 * @returns {Object.<x: number, y: number, z: number>}
 */
wbvector3.minus(vA, vB);
```

Returns the vectorial difference of the two vectors.


```
/**
 * @param {Object.<x: number, y: number, z: number>} v
 * @param {number} s
 * @returns {Object.<x: number, y: number, z: number>}
 */
wbvector3.multiply(v, s);
```

Multiplies the vector `v` by the scalar value `s` and returns the result.

```
/**
 * @param {Object.<x: number, y: number, z: number>} v
 * @returns {number}
 */
wbvector3.norm(v);
```

Returns the norm of the vector.

```
/**
 * @param {Object.<x: number, y: number, z: number>} vA
 * @param {Object.<x: number, y: number, z: number>} vB
 * @returns {number}
 */
wbvector3.distance(vA, vB);
```

Returns the shortest distance between the two vectors.

```
/**
 * @param {Object.<x: number, y: number, z: number>} vA
 * @param {Object.<x: number, y: number, z: number>} vB
 * @returns {Object.<x: number, y: number, z: number>}
 */
wbvector3.cross(vA, vB);
```

Returns the cross product between the two vectors.

```
/**
 * @param {Object.<x: number, y: number, z: number>} vA
 * @param {Object.<x: number, y: number, z: number>} vB
 * @returns {number}
 */
wbvector3.dot(vA, vB);
```

Returns the dot product between the two vectors.

```
/**
 * @param {Object.<x: number, y: number, z: number>} v
 * @returns {Object.<x: number, y: number, z: number>}
 */
wbvector3.normalize(v);
```

Normalizes the vector and returns the result.

%tab-end

%tab "wbmath"
```
/**
 * @param {number} angle
 * @returns {number}
 */
wbmath.degreesToRadians(angle);
```

Converts the provided number from degrees to radians.

```
/**
 * @param {number} angle
 * @returns {number}
 */
wbmath.radiansToDegrees(angle);
```

Converts the provided number from radians to degrees.

%tab-end

%tab "wbgeometry"
```
/**
 * @param {number} radius
 * @param {number} div
 * @param {Object.<x: number, y: number>} center
 * @param {number} shift
 * @returns {[Object.<x: number, y: number>, Object.<x: number, y: number>, ... ]}
 */
wbgeometry.circle(radius, div, c, shift);
```

Creates an array of `div` circle coordinates according to a circle of radius `radius` centered at `(center.x, center.y)` and rotated by `shift` radians.

```
/**
 * @param {Object.<x: number, y: number>} p
 * @param {[Object.<x: number, y: number>, Object.<x: number, y: number>, ... ]} polygon
 * @returns {boolean}
 */
wbgeometry.isPoint2InPolygon(p, polygon);
```

Returns true if point `p` is inside the provided polygon.
The polygon is defined as an array of objects with keys (x, y), each defining a vertex.

```
/**
 * @param {Object.<x: number, y: number>} reference
 * @param {[Object.<x: number, y: number>, Object.<x: number, y: number>, ... ]} points
 * @returns {Object.<x: number, y: number>}
 */
wbgeometry.findClosestPoint2InArray(reference, points);
```

Returns the closest point in the array to the reference point.
The array is comprised of objects with keys (x, y).

```
/**
 * @param {[Object.<x: number, y: number>, Object.<x: number, y: number>, ... ]} points
 * @returns {boolean}
 */
wbgeometry.isPoint2ArrayClockwise(points);
```

Returns true if an array of provided points is defined in a clockwise order.

```
/**
 * @param {[Object.<x: number, y: number>, Object.<x: number, y: number>, ... ]} points
 * @param {number} subdivision
 * @returns {[Object.<x: number, y: number>, Object.<x: number, y: number>, ... ]}
 */
wbgeometry.bSpline2(points, subdivision);
```

Creates a B-Spline curve of third order using the array of two-dimensional points, subdividing each segment by `subdivision` and returning the result.

```
/**
 * @param {[Object.<x: number, y: number, z: number>, Object.<x: number, y: number, z: number>, ... ]} points
 * @param {number} subdivision
 * @returns {[Object.<x: number, y: number, z: number>, Object.<x: number, y: number, z: number>, ... ]}
 */
wbgeometry.bSpline3(points, subdivision);
```

Creates a B-Spline curve of third order using the array of three-dimensional points, subdividing each segment by `subdivision` and returning the result.

%tab-end

%end

### Optimization

By default, PROTO files are considered to be deterministic.
That is, if the same procedural PROTO is instantiated multiple times in a world file and all the fields are the same in each case, then they will all generate the same result.
As such, any PROTO file with these properties only needs to be evaluated once, hence improving the loading performance.

If however a PROTO is supposed to generate different results, even when all the fields are the same, then these non-deterministic PROTO files should be labeled as such by adding a comment in the header of the file (i.e. `# tags: nonDeterministic`).
Typical cases of `nonDeterministic` PROTO files are those where the end result does not rely uniquely on the value of the fields but on something else (usually a randomly generated value).
`nonDeterministic` PROTO files are regenerated and therefore change at every reset.

> **Note**: when randomness is concerned, what defines determinism in a PROTO file, or lack of it, is the nature of the seed used by the random number generator.
Using a time-based seed (e.g. `wbrandom.seed(Date.now())`) or a seed based on the id of the node (e.g. `wbrandom.seed(context.id)`) are typical non-deterministic situations.
If the same seed is used every time or if it is not specified (i.e using the default seed), it leads instead to deterministic results.

### Example

```
TODO
```
