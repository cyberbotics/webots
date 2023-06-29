## Procedural PROTO Nodes

Procedural PROTO nodes can be created using [Lua](http://www.lua.org) as a scripting language.
Introducing and learning Lua is outside the scope of this document.
Please refer to the [Lua documentation](http://www.lua.org/docs.html) for complementary information.

> **Note**: Support for Lua as a scripting language will be discontinued in future releases, wherever possible [JavaScript](javascript-procedural-proto.md) should be preferred instead.

### Template Engine

A template engine is used to evaluate the PROTO according to the field values of the PROTO, before being loaded in Webots.
The template engine used is [liluat](https://github.com/FSMaxB/liluat) (under the MIT license).

### Programming Facts

- Using the template statements is exclusively allowed inside the content scope of the PROTO (cf. example).
- A template statement is encapsulated inside the "%{" and the "}%" tokens and can be written on several lines.
- Adding an "=" just after the opening token ("%{=") allows to evaluate a statement.
- The fields are accessible into a global Lua dictionary named "fields".
The dictionary keys matches the PROTO's fields names.
Each entry of this dictionary is a sub-dictionary with two keys named "value" and "defaultValue", the first one contains the current state of the field and the second one contains its the default state.
The conversion between the VRML97 types and the Lua types is detailed in [this table](#vrml97-type-to-lua-type-conversion).
- As shown in [this table](#vrml97-type-to-lua-type-conversion), the conversion of a VRML97 node is a Lua dictionary.
This dictionary contains the following keys: "node\_name" containing the VRML97 node name and "fields" which is a dictionary containing the Lua representation of the VRML97 node fields.
This dictionary is equal to `nil` if the VRML97 node is not defined (`NULL`).
For example, in the SimpleStairs example below, the `fields.appearance.node_name` key contains the `'Appearance'` string.
- The `context` dictionary provides contextual information about the PROTO.
Table [this table](#content-of-the-context-dictionary) shows the available information and its corresponding keys.
- The VRML97 comment ("#") prevails over the Lua statements.
- The following Lua modules are available directly: base, table, io, os, string, math, debug, package.
- The LUA\_PATH environment variable can be modified (before running Webots) to include external Lua modules.
- Lua standard output and error streams are redirected on the Webots console (written respectively in regular and in red colors).
This allows developers to use the Lua regular functions to write on these streams.
- The [lua-gd](http://ittner.github.io/lua-gd) module is contained in Webots and can simply be imported using `local gd = require("gd")`.
This module is very useful to manipulate images, it can be used, for example, to generate textures.

%figure "VRML97 type to Lua type conversion"

| VRML97 type  | Lua type                                              |
| ------------ | ----------------------------------------------------- |
| SFBool       | boolean                                               |
| SFInt32      | number                                                |
| SFFloat      | number                                                |
| SFString     | string                                                |
| SFVec2f      | dictionary (keys = "x" and "y")                       |
| SFVec3f      | dictionary (keys = "x", "y" and "z")                  |
| SFRotation   | dictionary (keys = "x", "y", "z" and "a")             |
| SFColor      | dictionary (keys = "r", "g" and "b")                  |
| SFNode       | dictionary (keys = "node\_name", "fields")            |
| MF*          | array (indexes = multiple value positions)            |

%end

%figure "Content of the context dictionary"

| Key                     | Value                                                                                                                                                                                                                                     |
| ----------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| world                   | absolute path to the current world file (including file name and extension)                                                                                                                                                               |
| proto                   | absolute path to the current PROTO file (including file name and extension)                                                                                                                                                               |
| project\_path           | absolute path to the current project directory                                                                                                                                                                                            |
| webots\_version         | dictionary representing the version of Webots with which the PROTO is currently used (dictionary keys: major and revision)                                                                                                                |
| webots\_home            | absolute path to the Webots installation directory                                                                                                                                                                                        |
| temporary\_files\_path  | absolute path to the temporary folder currently used by Webots (this is the location where the PROTO file is generated)                                                                                                                   |
| os                      | OS string ("windows", "linux" or "mac")                                                                                                                                                                                                   |
| id                      | id of the node. This id is equivalent to the one returned by the [wb\_supervisor\_node\_get\_id](supervisor.md#wb_supervisor_node_get_from_def) function and may be used for example to initialize the seed of a random number generator. |
| coordinate_system       | value of the [WorldInfo](worldinfo.md).`coordinateSystem` field.                                                                                                                                                                          |

%end

### Texture Generation

Using the [lua-gd](http://ittner.github.io/lua-gd) module it is possible to generate a texture image to be used by the PROTO.
The following standard fonts are available to write on the texture:

 - Arial
 - Arial Black
 - Comic Sans MS
 - Courier New
 - Georgia
 - Impact
 - Lucida Console
 - Lucida Sans Unicode
 - Palatino Linotype
 - Tahoma
 - Times New Roman
 - Trebuchet MS
 - Verdana

In addition to these fonts, it is possible to add other TrueType fonts file in your `PROJECT_HOME/fonts` directory.

### Optimization

By default, PROTO files are considered to be deterministic.
That is, if the same procedural PROTO is instantiated multiple times in a world file and all the fields are the same in each case, then they will all generate the same result.
As such, any PROTO file with these properties only needs to be evaluated once, hence improving the loading performance.

If however a PROTO is supposed to generate different results, even when all the fields are the same, then these non-deterministic PROTO files should be labeled as such by adding a comment in the header of the file (i.e. `# tags: nonDeterministic`).
Typical cases of `nonDeterministic` PROTO files are those where the end result does not rely uniquely on the value of the fields but on something else (usually a randomly generated value).
`nonDeterministic` PROTO files are regenerated and therefore change at every reset.

> **Note**: when randomness is concerned, what defines determinism in a PROTO file, or lack of it, is the nature of the seed used by the random number generator.
Using a time-based seed (e.g. `wbrandom.seed(os.clock() + os.time())`) or a seed based on the id of the node (e.g. `wbrandom.seed(context.id)`) are typical non-deterministic situations.
If the same seed is used every time or if it is not specified (i.e using the default seed), it leads instead to deterministic results.

### Example

```

#VRML_SIM R2019a utf8

PROTO SimpleStairs [
  field SFVec3f    translation 0 0 0
  field SFRotation rotation    0 0 1 0
  field SFString   name        "stairs"
  field SFInt32    nSteps      10
  field SFVec3f    stepSize    0.2 0.2 0.8
  field SFColor    color       0 1 0
  field SFString   text        "my text"
  field SFNode     physics     NULL
]
{
  # template statements can be used from here
  %{
    -- a template statement can be written on several lines
    if fields.nSteps.value < 1 then
      print('nSteps should be strictly positive')
    end

    -- print the path to this proto
    print(context.proto)

    if fields.stepSize.value.x ~= fields.stepSize.defaultValue.x then
      print('The X step size used is not the default one')
    end

    -- print the mass inside the physics node
    if fields.physics.value then
      print (fields.physics.value.fields.mass.value)
    end

   -- load lua-gd module and create a uniform texture
   local gd = require("gd")
   local debug = require("debug")
   local im = gd.createTrueColor(128, 128)
   color = im:colorAllocate(fields.color.value.r * 255, fields.color.value.g * 255, fields.color.value.b * 255)
   im:filledRectangle(0, 0, 127, 127, color)
   -- add the text in the texture
   textColor = im:colorAllocate(0, 0, 0)
   gd.fontCacheSetup()
   im:stringFT(textColor, "Arial", 20, 0, 5, 60, fields.text.value)
   -- save the image in a png file
   local name = debug.getinfo(1,'S').source  -- get the name of the current file
   name = name .. context.id  -- prevent name clashes
   local i = 0  -- make sure the file does not already exist
   local file = io.open(name .. i .. ".png", "r")
   while file do
     file:close()
     i = i + 1
     file = io.open(name .. i .. ".png", "r")
   end
   im:png(name .. i .. ".png")
   gd.fontCacheShutdown()
  }%
  Solid {
    translation IS translation
    rotation IS rotation
    children [
      DEF SIMPLE_STAIRS_GROUP Group {
        children [
        %{ for j = 0, (fields.nSteps.value - 1) do }%
          %{ x = j * fields.stepSize.value.x }%
          %{ y = j * fields.stepSize.value.y + fields.stepSize.value.y / 2 }%
            Pose {
              translation %{=x}% %{=y}% 0
              children [
                Shape {
                  appearance Appearance {
                    texture ImageTexture {
                      url [ %{= '"' .. context.temporary_files_path .. name .. i .. '.png"' }% ]
                    }
                  }
                  geometry Box {
                    size IS stepSize
                  }
                }
              ]
            }
          %{ end }%
        ]
      }
    ]
    name IS name
    boundingObject USE SIMPLE_STAIRS_GROUP
    physics IS physics
  }
  # template statements can be used up to there
}
```
