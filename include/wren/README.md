### Minimal interface between Webots and Wren

Wren exposes its public interface through a series of C headers, found in the include/wren directory

#### Wren dependencies

Wren requires only on these 4 dependencies:

1. STL
2. glm
3. glade
4. siphash

In particular, the dependency on Qt has been removed.
