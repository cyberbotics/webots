### Differences between Webots and Wren

Here we try to document things that WREN does differently from Webots and that may require changes to existing worlds.

#### IndexedLineSet

Before, the IndexedLineSet was shaded in the same way as all other geometries. This often results in a different
color for the lines than the material's diffuse color.
In the WREN branch, the IndexedLineSet uses a separate shader that directly applies the material's diffuse color.
