#version 330

const int bgTextureIndex = 0;
const int mainTextureIndex = 1;
const int fgTextureIndex = 2;
const int closeButtonTextureIndex = 3;
const int resizeButtonTextureIndex = 4;

in float aspectRatio;
in float closeButtonProportionX;
in float closeButtonProportionY;
in float resizeButtonProportionX;
in float resizeButtonProportionY;

in vec2 texUv;
in vec2 texUvFrame;

out vec4 fragColor;

uniform int channelCount;

uniform sampler2D inputTextures[5];

layout(std140) uniform Overlay {
  vec4 positionAndSize;
  vec4 defaultSize;  // x,y: size, z: render default size instead of actual overlay
  vec4 borderColor;
  vec4 backgroundColor;
  vec4 activeFlags;   // x: bg texture, y: main texture, z: fg texture, w: border
  vec4 textureFlags;  // x: flip vertically, y: additional texture count, z: maxRange (depth textures only),
                      // w: overlay transparency
  vec2 sizeInPixels;  // x,y: size in screen pixels
  vec2 borderSize;    // x: vertical size, y: horizontal size
}
overlay;

void main() {
  fragColor = vec4(0.0);

  // render default size indicator if requested
  if (overlay.defaultSize.z > 0.0) {
    if (overlay.positionAndSize.z > overlay.defaultSize.x || overlay.positionAndSize.w > overlay.defaultSize.y)
      fragColor = vec4(1.0, 1.0, 1.0, 0.3);
    else
      fragColor = vec4(0.4, 0.4, 0.4, 0.3);
  } else if (texUv.x < 0.0 || texUv.x > 1.0 || texUv.y < 0.0 || texUv.y > 1.0)
    fragColor = overlay.borderColor;
  else {
    // bg texture
    if (overlay.activeFlags.x > 0.0)
      fragColor = vec4(texture(inputTextures[bgTextureIndex], texUv).rgb, 1.0);
    else
      fragColor = vec4(overlay.backgroundColor);

    // main texture
    if (overlay.activeFlags.y > 0.0) {
      vec4 color = texture(inputTextures[mainTextureIndex], texUv);

      // normalize depth if required
      if (overlay.textureFlags.z > 0.0f)
        color.x = color.x / overlay.textureFlags.z;

      if (channelCount == 1)
        color = vec4(color.x, color.x, color.x, 1.0);

      if (overlay.textureFlags.w == 0.0)
        color = vec4(color.xyz, 1.0);

      fragColor = mix(fragColor, color, color.a);
    }

    // fg texture
    if (overlay.activeFlags.z > 0.0) {
      vec4 color = texture(inputTextures[fgTextureIndex], texUv);
      fragColor = mix(fragColor, color, color.a);
    }

    // close button
    if (overlay.textureFlags.y > 0.0) {
      float closeButtonStartX = (1.0 - closeButtonProportionX) + overlay.borderSize.x;
      float closeButtonStartY = (1.0 - closeButtonProportionY) + overlay.borderSize.y;

      if (texUvFrame.x >= closeButtonStartX && texUvFrame.y >= closeButtonStartY) {
        vec2 closeButtonTexUv = texUvFrame - vec2(closeButtonStartX, closeButtonStartY);
        closeButtonTexUv *= vec2(1.0 / closeButtonProportionX, 1.0 / closeButtonProportionY);
        vec4 color = texture(inputTextures[closeButtonTextureIndex], clamp(closeButtonTexUv, 0.0, 1.0));
        fragColor = mix(fragColor, color, color.a);
      }
    }

    // resize button
    if (overlay.textureFlags.y > 1.0) {
      float resizeButtonStartX = 1.0 - resizeButtonProportionX;
      float resizeButtonEndY = resizeButtonProportionY;

      if (texUvFrame.x > resizeButtonStartX && texUvFrame.y < resizeButtonEndY) {
        vec2 resizeButtonTexUv = texUvFrame - vec2(resizeButtonStartX, 0.0);
        resizeButtonTexUv *= vec2(1.0 / resizeButtonProportionX, 1.0 / resizeButtonProportionY);
        vec4 color = texture(inputTextures[resizeButtonTextureIndex], clamp(resizeButtonTexUv, 0.0, 1.0));
        fragColor = mix(fragColor, color, color.a);
      }
    }
  }
}
