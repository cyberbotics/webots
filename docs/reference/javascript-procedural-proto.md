## Procedural PROTO Nodes

The descriptive power of PROTO files can be significantly extended by generating object in a procedural way.
Procedural PROTO nodes can be created using JavaScript as a scripting language.
Introducing and learning JavaScript is outside the scope of this document.

### Template Engine

What determines if a PROTO file is procedural or not, is whether template statements exist in its body, in other words if JavaScript expressions are encapsulated either between the tokens `%<` and `>%` or `%<=` and `>%`.
Webots cannot load procedural PROTO nodes directly, therefore the procedural PROTO is first translated to pure JavaScript and the resulting script is evaluated by a JavaScript engine, in Webot's case using QJSEngine, and the result of this process yields a VRML97 compatible PROTO file which is then loaded by Webots as any other non-procedural would.

### Programming Facts

- For backwards compatibility reasons, procedural PROTO files are assumed to be Lua by default.
In order to use JavaScript scripting it is required to add the comment `# template language: javascript` in the header of the file.
- A template statement is encapsulated inside the `%<` and the `>%` tokens and can be written on several lines.
- Adding an "=" just after the opening token (`%<=`) allows to evaluate a statement.
- The use of template statements is exclusively allowed inside the content scope of the PROTO (cf. example).
- The fields are accessible in a globally accessible object named "fields".
The keys of this object match the PROTO's field names.
For each field, an additional sub-object with two keys named "value" and "defaultValue" is available.
The first represents the effective value of the field (for instance the one defined in the world file) and the second defines the default value (i.e the one specified in the PROTO header of the file).

> **Note**: For a field defined in the PROTO header as `field SFFloat numberOfPoints 10`, they can be retrieved by calling `fields.numberOfPoints.value` and `field.numberOfPoints.defaultValue`.

- The conversion between the VRML97 types and the JavaScript types is detailed in [this table](#vrml97-type-to-javascript-type-conversion).
- As shown in [this table](#vrml97-type-to-javascript-type-conversion), the conversion of a VRML97 node is an object.
This object contains the following keys: "node\_name" containing the VRML97 node name and "fields" which is in turn an object containing the JavaScript representation of the VRML97 node fields.
This object is equal to `undefined` if the VRML97 node is not defined (`NULL`).
- Objects that are part of [ECMA-262](http://www.ecma-international.org/publications/standards/Ecma-262.htm) are built-in and globally accessible, such as `Math`, `Date` and `String`.
- The `context` object provides contextual information about the PROTO.
Table [this table](#content-of-the-context-object) shows the available information and the corresponding keys.
- The VRML97 comment ("#") prevails over the JavaScript statements.
- JavaScript standard output and error streams are redirected on the Webots console (written respectively in regular and in red colors).
Developers can use `console.log`, `console.warn`, `console.debug` and `console.error` to write on these streams.
- The resulting JavaScript script is evaluated in "strict mode", so particular care must be taken to respect it.

#### Good Practices and Common Pitfalls

- When using template statements (i.e `%<` and `>%`) with constructs such as `for` loops and `if` conditionals particular care must be taken with regards to the brackets.
It is technically legitimate to forego the brackets for one-line statements, however when these are embedded in the body of a PROTO, there is no guarantee that in the evaluation process only one statement effectively exists between them.
Consider the example below, the option on the left would technically work however if the radius is later changed to something like `radius %<= fields.radius.value >%` it no longer would because the parsing of this expression involves several steps.
It is therefore encouraged to be verbose and provide the brackets.

%tab-component "generic"

%tab "Technically correct, but risky"
```
Sphere {
  %< if (fields.condition.value) >%
      radius 1
  %< else >%
      radius 2
}
```
%tab-end

%tab "Safer alternative"
```
Sphere {
  %< if (fields.condition.value) { >%
      radius 1
  %< } else { >%
      radius 2
  %< } >%
}
```
%tab-end

%end

- Although not mandatory, the usage of semi-colons for JavaScript statements is highly encouraged.
- Lua and JavaScript Procedural PROTO nodes use two distinct tokens (`%{` and `}%` for Lua and `%<` and `>%` for JavaScript) and cannot be interchanged.
Which tokens will be considered depends on whether the comment line `# template language: javascript` is present.
- The `wbfile` module for file manipulation does not need to, and should not, be imported as it is added automatically to each instance of the engine.
- Performance degradation has been observed when the number of evaluations requested (i.e expressions of the form `%<= ... >%`) is large, generally in the tens of thousands.
This is typically the case when expressions of this form are used to define the coordinates or indexes of, for instance, a [IndexedFaceSet](indexedfaceset.md).
To greatly speed-up the generation of this sort of PROTO file, it is highly suggested to use a string buffer to which the coordinates are progressively appended and to only evaluate this buffer once at the end, as shown in the following snippet.

%tab-component "generic"

%tab "Technically correct, but slow"
```
geometry IndexedFaceSet {
  coord Coordinate {
    point [
      %< for (let i = 0; i < 10000; ++i) { >%
          %<= i + 0 >% %<= i + 1 >% %<= i + 2 >%
      %< } >%
    ]
  }

  ...
}
```
%tab-end

%tab "Faster alternative"
```
geometry IndexedFaceSet {
  coord Coordinate {
    point [
      %<
        let pointBuffer = '';
        for (let i = 0; i < 10000; ++i)
          pointBuffer += (i + 0) + ' ' + (i + 1) + ' ' + (i + 2) + '\n';
      >%
      %<= pointBuffer >%
    ]
  }

  ...
}
```
%tab-end

%end

#### VRML97 Type Conversion

%figure "VRML97 type to JavaScript type conversion"

| VRML97 type  | JavaScript type                                       | Example                           |
| ------------ | ----------------------------------------------------- | ---------------------------       |
| SFBool       | Boolean                                               | true                              |
| SFInt32      | Number                                                | 10                                |
| SFFloat      | Number                                                | 10.0                              |
| SFString     | String                                                | 'Webots'                          |
| SFVec2f      | Object (keys: "x" and "y")                            | {x: 1, y: 2}                      |
| SFVec3f      | Object (keys: "x", "y" and "z")                       | {x: 1, y: 2, z: 3}                |
| SFRotation   | Object (keys: "x", "y", "z" and "a")                  | {x: 1, y: 2, z: 3, a: 3.14}       |
| SFColor      | Object (keys: "r", "g" and "b")                       | {r: 1, g: 0, b: 0}                |
| SFNode       | Object (keys: "node\_name", "fields")                 | {node\_name: 'Box', fields: ... } |
| MF*          | Array (indexes = multiple value positions)            | [... , ... , ...]                 |

%end

%figure "Content of the context object"

| Key                     | Value                                                                                                                                                                                                                                     |
| ----------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| world                   | absolute path to the current world file (including file name and extension)                                                                                                                                                               |
| proto                   | absolute path to the current PROTO file (including file name and extension)                                                                                                                                                               |
| project\_path           | absolute path to the current project directory                                                                                                                                                                                            |
| webots\_version         | object representing the version of Webots with which the PROTO is currently used (keys: "major" and "revision")                                                                                                                |
| webots\_home            | absolute path to the Webots installation directory                                                                                                                                                                                        |
| temporary\_files\_path  | absolute path to the temporary folder currently used by Webots                                                                                                                 |
| os                      | OS string ("windows", "linux" or "mac")                                                                                                                                                                                                   |
| id                      | id of the node. This id is equivalent to the one returned by the [wb\_supervisor\_node\_get\_id](supervisor.md#wb_supervisor_node_get_from_def) function and may be used for example to initialize the seed of a random number generator. |
| coordinate_system       | value of the [WorldInfo](worldinfo.md).`coordinateSystem` field.                                                                                                                                                                          |

%end

#### Utility Modules

A number of modules provide additional utility functions that can be useful when creating procedural PROTO files.
To use these functions, the module needs to be included first:

```javascript
%<
  // to import the entire module
  import * as wbrandom from 'wbrandom.js';

  // access functions
  let number = wbrandom.integer(0, 10);

  // to import only specific functions instead of the entire module
  import {integer} from 'wbrandom.js';
>%
```

The available modules are the following:

- `wbrandom`: provides functions that allow seed-based pseudo-random number generation.

- `wbrotation`: provides utility functions for dealing with rotations, represented in axis-angle, quaternions or matrix format.

- `wbvector2`: provides utility functions to manipulate two-dimensional vectors.

- `wbvector3`: provides utility functions to manipulate three-dimensional vectors.

- `wbgeometry`: provides geometry related functions, allowing for instance to create splines.

- `wbutility`: provides commonly needed functions.

Additionally, the following module is automatically loaded to the engine and therefore does not need to be imported:

- `wbfile`: provides functions for the reading and writing of files.

> **Note:**: contrary to the other JavaScript modules, `wbfile` is a C++ wrapped class and therefore cannot and should not be imported manually, attempting to do so will return an error.
The functions exported by this module are available globally.

The functions exported by each module are:

%tab-component "generic"

%tab "wbrandom"
```javascript
/**
 * @param {Number} s
 */
wbrandom.seed(s);
```

Sets the random number generator seed to the specified value.
The numbers are generated using a Linear Congruential Generator (LCG) algorithm.
To generate non-deterministic results, a time-based seed can be used `wbrandom.seed(Date.now())`.

```javascript
/**
 * @param {Number} [min]
 * @param {Number} [max]
 * @returns {Number}
 */
wbrandom.real(min, max);
```

Returns a floating point pseudo-random number in the range [min, max].
If only one parameter is provided, the number will be between zero and that value.
If no parameters are provided, it returns a value in the range [0, 1].

```javascript
/**
 * @param {Number} [min]
 * @param {Number} [max]
 * @returns {Number}
 */
wbrandom.integer(min, max);
```

Returns a pseudo-random integer number in the range [min, max].
If only one parameter is provided, the number will be between zero and it.
If no parameters are provided, it returns a value in the range [0, 2^24 - 1].

%tab-end

%tab "wbrotation"
```javascript
/**
 * @param {Object.<x: number, y: number, z: number, a: number>} rA
 * @param {Object.<x: number, y: number, z: number, a: number>} rB
 * @returns {Boolean}
 */
wbrotation.equal(rA, rB);
```

Returns true if the two rotation vectors, in axis-angle format, are equal.
Returns false otherwise.

```javascript
/**
 * @param {Object.<w: number, x: number, y: number, z: number>} q
 * @returns {Object.<x: number, y: number, z: number, a: number>}
 */
wbrotation.fromQuaternion(q);
```

Converts a rotation provided as quaternion to an axis-angle representation.

```javascript
/**
 * @param {Object.<x: number, y: number, z: number, a: number>} r
 * @returns {Object.<w: number, x: number, y: number, z: number>}
 */
wbrotation.toQuaternion(r);
```

Converts a rotation in axis-angle representation to quaternion format.

```javascript
/**
 * @param {Object.<0: number, 1: number, ..., 8: number>} m
 * @returns {Object.<x: number, y: number, z: number, a: number>}
 */
wbrotation.fromMatrix3(m);
```

Converts a rotation in matrix form (3 x 3) to an axis-angle representation.

```javascript
/**
 * @param {Object.<x: number, y: number, z: number, a: number>} r
 * @returns {Object.<0: number, 1: number, ..., 8: number>}
 */
wbrotation.toMatrix3(r);
```

Converts a rotation in axis-angle representation to matrix (3 x 3) format.

```javascript
/**
 * @param {Object.<x: number, y: number, z: number, a: number>} r
 * @returns {Boolean}
 */
wbrotation.isIdentity(r);
```

Returns true if the rotation provided in axis-angle representation corresponds to the identity matrix.

```javascript
/**
 * @param {Object.<x: number, y: number, z: number, a: number>} r
 * @returns {Object.<x: number, y: number, z: number, a: number>}
 */
wbrotation.normalize(r);
```

Normalizes both the axis and the angle of the provided rotation vector.

```javascript
/**
 * @param {Object.<x: number, y: number, z: number, a: number>} rA
 * @param {Object.<x: number, y: number, z: number, a: number>} rB
 * @returns {Object.<x: number, y: number, z: number, a: number>}
 */
wbrotation.combine(rA, rB);
```

Combines two rotations, in axis-angle format, together.

```javascript
/**
 * @param {Object.<0: number, 1: number, ..., 8: number>} m
 * @param {Object.<x: number, y: number, z: number>} v
 * @returns {Object.<x: number, y: number, z: number>}
 */
wbrotation.rotateVector3ByMatrix3(m, v);
```

Rotates the 3-dimensional vector `v` according to the rotation matrix `m`.

```javascript
/**
 * @param {Object.<x: number, y: number, z: number, a: number>} r
 * @param {Object.<x: number, y: number, z: number>} v
 * @returns {Object.<x: number, y: number, z: number>}
 */
wbrotation.rotateVector3ByRotation(r, v);
```

Rotates the 3-dimensional vector `v` according to the rotation vector `r`.

```javascript
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
```javascript
/**
 * @param {Object.<x: number, y: number>} vA
 * @param {Object.<x: number, y: number>} vB
 * @returns {Boolean}
 */
wbvector2.equal(vA, vB);
```

Returns true if the two vectors are equal.

```javascript
/**
 * @param {Object.<x: number, y: number>} vA
 * @param {Object.<x: number, y: number>} vB
 * @returns {Object.<x: number, y: number>}
 */
wbvector2.add(vA, vB);
```

Returns the vectorial sum of the two vectors.

```javascript
/**
 * @param {Object.<x: number, y: number>} vA
 * @param {Object.<x: number, y: number>} vB
 * @returns {Object.<x: number, y: number>}
 */
wbvector2.minus(vA, vB);
```

Returns the vectorial difference of the two vectors.

```javascript
/**
 * @param {Object.<x: number, y: number>} v
 * @param {Number} s
 * @returns {Object.<x: number, y: number>}
 */
wbvector2.multiply(v, s);
```

Multiplies the vector `v` by the scalar `s` and returns the result.

```javascript
/**
 * @param {Object.<x: number, y: number>} v
 * @returns {Number}
 */
wbvector2.norm(v);
```

Returns the norm of the provided vector.

```javascript
/**
 * @param {Object.<x: number, y: number>} vA
 * @param {Object.<x: number, y: number>} vB
 * @returns {Number}
 */
wbvector2.distance(vA, vB);
```

Returns the shortest distance between the two vectors.

```javascript
/**
 * @param {Object.<x: number, y: number>} vA
 * @param {Object.<x: number, y: number>} vB
 * @returns {Number}
 */
wbvector2.angle(vA, vB);
```

Returns the angle between the two vectors.

```javascript
/**
 * @param {Object.<x: number, y: number>} vA
 * @param {Object.<x: number, y: number>} vB
 * @returns {Number}
 */
wbvector2.cross(vA, vB);
```

Returns the cross product between the two vectors.

```javascript
/**
 * @param {Object.<x: number, y: number>} vA
 * @param {Object.<x: number, y: number>} vB
 * @returns {Number}
 */
wbvector2.dot(vA, vB);
```

Returns the dot product between the two vectors.

```javascript
/**
 * @param {Object.<x: number, y: number>} v
 * @returns {Object.<x: number, y: number>}
 */
wbvector2.normalize(v);
```

Normalizes the provided vector.

```javascript
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
```javascript
/**
 * @param {Object.<x: number, y: number, z: number>} vA
 * @param {Object.<x: number, y: number, z: number>} vB
 * @returns {Boolean}
 */
wbvector3.equal(vA, vB);
```

Returns true if the two 3-dimensional vectors are equal.

```javascript
/**
 * @param {Object.<x: number, y: number, z: number>} vA
 * @param {Object.<x: number, y: number, z: number>} vB
 * @returns {Object.<x: number, y: number, z: number>}
 */
wbvector3.add(vA, vB);
```

Returns the vectorial sum of the two vectors.

```javascript
/**
 * @param {Object.<x: number, y: number, z: number>} vA
 * @param {Object.<x: number, y: number, z: number>} vB
 * @returns {Object.<x: number, y: number, z: number>}
 */
wbvector3.minus(vA, vB);
```

Returns the vectorial difference of the two vectors.


```javascript
/**
 * @param {Object.<x: number, y: number, z: number>} v
 * @param {Number} s
 * @returns {Object.<x: number, y: number, z: number>}
 */
wbvector3.multiply(v, s);
```

Multiplies the vector `v` by the scalar value `s` and returns the result.

```javascript
/**
 * @param {Object.<x: number, y: number, z: number>} v
 * @returns {Number}
 */
wbvector3.norm(v);
```

Returns the norm of the vector.

```javascript
/**
 * @param {Object.<x: number, y: number, z: number>} vA
 * @param {Object.<x: number, y: number, z: number>} vB
 * @returns {Number}
 */
wbvector3.distance(vA, vB);
```

Returns the shortest distance between the two vectors.

```javascript
/**
 * @param {Object.<x: number, y: number, z: number>} vA
 * @param {Object.<x: number, y: number, z: number>} vB
 * @returns {Object.<x: number, y: number, z: number>}
 */
wbvector3.cross(vA, vB);
```

Returns the cross product between the two vectors.

```javascript
/**
 * @param {Object.<x: number, y: number, z: number>} vA
 * @param {Object.<x: number, y: number, z: number>} vB
 * @returns {Number}
 */
wbvector3.dot(vA, vB);
```

Returns the dot product between the two vectors.

```javascript
/**
 * @param {Object.<x: number, y: number, z: number>} v
 * @returns {Object.<x: number, y: number, z: number>}
 */
wbvector3.normalize(v);
```

Normalizes the vector and returns the result.

%tab-end

%tab "wbgeometry"
```javascript
/**
 * @param {Number} radius
 * @param {Number} div
 * @param {Object.<x: number, y: number>} center
 * @param {Number} shift
 * @returns {[Object.<x: number, y: number>, Object.<x: number, y: number>, ... ]}
 */
wbgeometry.circle(radius, div, c, shift);
```

Creates an array of `div` circle coordinates according to a circle of radius `radius` centered at `(center.x, center.y)` and rotated by `shift` radians.

```javascript
/**
 * @param {Object.<x: number, y: number>} p
 * @param {[Object.<x: number, y: number>, Object.<x: number, y: number>, ... ]} polygon
 * @returns {Boolean}
 */
wbgeometry.isPoint2InPolygon(p, polygon);
```

Returns true if point `p` is inside the provided polygon.
The polygon is defined as an array of objects with keys (x, y), each defining a vertex.

```javascript
/**
 * @param {Object.<x: number, y: number>} reference
 * @param {[Object.<x: number, y: number>, Object.<x: number, y: number>, ... ]} points
 * @returns {Object.<x: number, y: number>}
 */
wbgeometry.findClosestPoint2InArray(reference, points);
```

Returns the closest point in the array to the reference point.
The array is comprised of objects with keys (x, y).

```javascript
/**
 * @param {[Object.<x: number, y: number>, Object.<x: number, y: number>, ... ]} points
 * @returns {Boolean}
 */
wbgeometry.isPoint2ArrayClockwise(points);
```

Returns true if an array of provided points is defined in a clockwise order.

```javascript
/**
 * @param {[Object.<x: number, y: number>, Object.<x: number, y: number>, ... ]} points
 * @param {Number} subdivision
 * @returns {[Object.<x: number, y: number>, Object.<x: number, y: number>, ... ]}
 */
wbgeometry.bSpline2(points, subdivision);
```

Creates a B-Spline curve of third order using the array of two-dimensional points, subdividing each segment by `subdivision` and returning the result.

```javascript
/**
 * @param {[Object.<x: number, y: number, z: number>, Object.<x: number, y: number, z: number>, ... ]} points
 * @param {Number} subdivision
 * @returns {[Object.<x: number, y: number, z: number>, Object.<x: number, y: number, z: number>, ... ]}
 */
wbgeometry.bSpline3(points, subdivision);
```

Creates a B-Spline curve of third order using the array of three-dimensional points, subdividing each segment by `subdivision` and returning the result.

%tab-end

%tab "wbutility"
```javascript
/**
 * @param {Boolean} statement
 * @param {String} message
 */
wbutility.assert(statement, message);
```

If the statement is false, it prints `message` to the standard error stream `stderr`.

```javascript
/**
 * @param {String} message
 */
wbutility.error(message);
```

It writes `message` to the standard error stream `stderr`.

```javascript
/**
 * @param {String} message
 */
wbutility.info(message);
```

It writes `message` to the standard output stream `stdout`.

```javascript
/**
 * @param {Object} original
 * @returns {Object}
 */
wbutility.deepCopy(message);
```

Creates and returns a deep copy of any object provided as argument (i.e an independent clone).

```javascript
/**
 * @param {Number} angle
 * @returns {Number}
 */
wbutility.degreesToRadians(angle);
```

Converts the provided number from degrees to radians.

```javascript
/**
 * @param {Number} angle
 * @returns {Number}
 */
wbutility.radiansToDegrees(angle);
```

Converts the provided number from radians to degrees.

```javascript
/**
 * @param {Number} s
 * @returns {Boolean}
 */
wbutility.isScalar(s);
```

Returns true if the provided argument is a scalar.

```javascript
/**
 * @param {Object<x: number, y: number} v
 * @returns {Boolean}
 */
wbutility.isVector2(v);
```

Returns true if the provided argument is a vector2, i.e if it is a two-dimensional object with keys (x and y).

```javascript
/**
 * @param {Object<x: number, y: number, z: number} v
 * @returns {Boolean}
 */
wbutility.isVector3(v);
```

Returns true if the provided argument is a vector3, i.e if it is a three-dimensional object with keys (x, y and z).

```javascript
/**
 * @param {Object<x: number, y: number, z: number, a: number} r
 * @returns {Boolean}
 */
wbutility.isAxisAngle(r);
```

Returns true if the provided argument is a vector in axis-angle format, i.e if it is a four-dimensional object with keys (x, y, z and a).

```javascript
/**
 * @param {Object<w: number, x: number, y: number, z: number} q
 * @returns {Boolean}
 */
wbutility.isQuaternion(q);
```

Returns true if the provided argument is a quaternion, i.e if it is a four-dimensional object with keys (w, x, y and z).

```javascript
/**
 * @param {Object<0: number, 1: number, ... , 8: number} m
 * @returns {Boolean}
 */
wbutility.isMatrix3(m);
```

Returns true if the provided argument is a three-dimensional matrix, i.e if it is a 9-dimensional object with keys (0, 1, ..., 8).

```javascript
/**
 * @param {[{Object<x: number, y: number, [z: number]}, {Object<x: number, y: number, [z: number]}, ...]} array
 * @param {Number} dim
 * @returns {Boolean}
 */
wbutility.isArrayOfPoints(array, dim);
```

Returns true if the provided argument is an array of `dim`-dimensional points.
`dim` can either be 2 or 3.

%tab-end

%tab "wbfile"

```javascript
/**
 * @param {String} filePath;
 * @returns {Boolean}
 */
wbfile.fileExists(filePath);
```

Returns true if the file specified by the path exists, or false otherwise.

```javascript
/**
 * @param {String} filePath;
 * @returns {String}
 */
wbfile.readTextFile(filePath);
```

Opens the file available at `filePath` and returns its contents.

```javascript
/**
 * @param {String} fileName;
 * @param {String} content;
 * @returns {Boolean}
 */
wbfile.writeTextFile(fileName, content);
```

Writes the provided string to a file of name `fileName`.
The file will be saved in the temporary file path.
The location of this path can be retrieved from the `context` field object, see [this table](#content-of-the-context-object).

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
#VRML_SIM {{ webots.version.major }} utf8
# tags: nonDeterministic
# template language: javascript

PROTO DominoSpawner [
  # these are the PROTO fields and define the default values. Here template statements are not allowed
  field SFVec2f   startPoint          0 0
  field SFVec2f   endPoint            1 1
  field SFInt32   numberOfDominos     10
  field SFFloat   randomColorSeed     -1
  field SFVec3f   dominoShape         0.02 0.16 0.06
  field MFColor   colorSet            [1 0 0, 0 1 0, 0 0 1]
  field SFString  playerName          "stranger"
]
{
  # template statements can be used from here
  %<
    // use additional modules for extra functionality
    import * as wbgeometry from 'wbgeometry.js';
    import * as wbrandom from 'wbrandom.js';
    import * as wbvector2 from 'wbvector2.js';

    // print a welcoming message to the Webots console
    let name = fields.playerName.value;
    console.log('Hello ' + name + '!' );

    let seed = fields.randomColorSeed.value;
    if (seed === -1)
      // use a time-based seed for the random number generator. This makes the PROTO non-deterministic
      wbrandom.seed(Date.now());
    else
      // use a specific seed everytime, this makes the PROTO deterministic (will be identical everytime)
      wbrandom.seed(seed);

    // ensure field values are acceptable, otherwise overwrite them
    let numberOfDominos = fields.numberOfDominos.value;
    if (numberOfDominos < 2)
      numberOfDominos = fields.numberOfDominos.defaultValue;

    let shape = fields.dominoShape.value;
    if (shape.x <= 0 || shape.y <= 0 || shape.z <= 0) {
      shape = fields.dominoShape.defaultValue;
      console.error('The sides of the domino must be strictly positive.');
    }

    // determine distance between dominos using utility functions from the wbvector2 module
    const startPoint = fields.startPoint.value;
    const endPoint = fields.endPoint.value;

    const distanceStep = wbvector2.norm(wbvector2.minus(endPoint, startPoint)) / (numberOfDominos - 1);

    // determine the orientation of the row of dominos
    const angle = wbvector2.angle(startPoint, endPoint);

    // generate the properties of domino set
    let dominoSet = [];

    for (let i = 0; i < numberOfDominos; ++i) {
      // determine the position of the domino
      let coordinates = {x: startPoint.x + i * distanceStep * Math.cos(angle), y: 0, z: startPoint.y + i * distanceStep * Math.sin(angle)};

      // select a random color from the colorSet
      let colorSet = fields.colorSet.value;
      let index = wbrandom.integer(0, colorSet.length - 1);
      let color = colorSet[index];

      dominoSet.push({coordinates: coordinates, color: color});
    }
  >%
  Group {
    children [
      %< for (let i = 0; i < dominoSet.length; ++i) { >%
        Pose {
          translation %<= dominoSet[i].coordinates.x >% %<= dominoSet[i].coordinates.y >% %<= dominoSet[i].coordinates.z >%
          rotation 0 1 0 %<= Math.PI - angle >%
          children [
            Shape {
              appearance PBRAppearance {
                baseColor %<= dominoSet[i].color.r >% %<= dominoSet[i].color.g >% %<= dominoSet[i].color.b >%
              }
              geometry Box {
                size %<= shape.x >% %<= shape.y >% %<= shape.z >%
              }
            }
          ]
        }
    %< } >%
    ]
  }

  # template statements can be used up to there
}
```
