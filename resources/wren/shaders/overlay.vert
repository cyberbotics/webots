#version 330 core

layout(location = 0) in vec3 vCoord;
layout(location = 2) in vec2 vTexCoord;

const float closeButtonSize = 14.0;
const float resizeButtonSize = 16.0;

out float aspectRatio;
out float closeButtonProportionX;
out float closeButtonProportionY;
out float resizeButtonProportionX;
out float resizeButtonProportionY;

// Two sets of texcoords are needed since main texture may be flipped
out vec2 texUv;
out vec2 texUvFrame;

layout(std140) uniform Overlay {
  vec4 positionAndSize;  // in percentage of OpenGL viewport size
  vec4 defaultSize;      // x,y: size, z: render default size instead of actual overlay
  vec4 borderColor;
  vec4 backgroundColor;
  vec4 textureFlags;  // x: flip vertically, y: additional texture count, z: maxRange (depth textures only),
                      // w: overlay transparency
  uvec2 activeFlags;  // x: textures, y: border
  vec2 sizeInPixels;  // x,y: size in screen pixels
  vec2 borderSize;    // x: vertical size, y: horizontal size in percentage of OpenGL viewport size
}
overlay;

void main() {
  aspectRatio = overlay.sizeInPixels.x / overlay.sizeInPixels.y;
  closeButtonProportionX = closeButtonSize / overlay.sizeInPixels.x;
  closeButtonProportionY = aspectRatio * closeButtonProportionX;
  resizeButtonProportionX = resizeButtonSize / overlay.sizeInPixels.x;
  resizeButtonProportionY = aspectRatio * resizeButtonProportionX;

  texUv = vTexCoord;

  // render default size if requested
  vec2 actualSize = overlay.positionAndSize.zw;
  if (overlay.defaultSize.z > 0.0)
    actualSize = overlay.defaultSize.xy;

  // if texcoords are not in [0.0, 1.0], the border color will be used in the fragment shader
  vec2 sizeWithBorder = actualSize;
  if (overlay.activeFlags.y != 0u) {
    sizeWithBorder += 2.0 * overlay.borderSize;
    vec2 scaleFactor = sizeWithBorder / actualSize;
    texUv -= vec2(0.5);
    texUv *= scaleFactor;
    texUv += vec2(0.5);
  }

  // vCoord.xy go from (0.0, 0.0) to (1.0, 1.0) (see definition of Quad mesh).
  // overlay.positionAndSize.y goes from 0.0 (top) to 1.0 (bottom),
  // which is the inverse direction of OpenGL clip space Y.
  // First scale and translate the coordinates to be inside [(-1.0, -1.0), (1.0, 1.0)],
  // then apply the offsets while inverting the Y axis.
  vec2 finalPosition = 2.0 * sizeWithBorder * vec2(vCoord.x, vCoord.y);
  finalPosition -= 1.0;
  finalPosition += 2.0 * vec2(overlay.positionAndSize.x,                          // x offset
                              1.0 - sizeWithBorder.y - overlay.positionAndSize.y  // y offset
                         );

  gl_Position = vec4(finalPosition, 0.0, 1.0);

  texUvFrame = texUv;
  // flip texture vertically if requested
  if (overlay.textureFlags.x > 0.0)
    texUv.y = 1.0 - texUv.y;
}
