#version 330 core

precision highp float;

#define pi_2 1.570796327

// These values should coincide with the ones of WbWrenCamera::CameraOrientation
#define FRONT 0
#define RIGHT 1
#define BACK 2
#define LEFT 3
#define UP 4
#define DOWN 5

const float FLT_MAX = intBitsToFloat(0x7F800000);

const vec3 orientations[6] = vec3[6](vec3(1.0, 0.0, 0.0), vec3(0.0, -1.0, 0.0), vec3(-1.0, 0.0, 0.0), vec3(0.0, 1.0, 0.0),
                                     vec3(0.0, 0.0, 1.0), vec3(0.0, 0.0, -1.0));

in vec2 texUv;

out vec4 fragColor;

uniform bool rangeCamera;
uniform bool cylindrical;

uniform float maxRange;
uniform float minRange;
uniform float fovX;
uniform float fovY;
uniform float fovYCorrectionCoefficient;

uniform sampler2D inputTextures[6];

// Same as texture but pick the side to used for the interpolation to avoid discontinuities
vec4 textureSmooth(sampler2D tex, vec2 p, int bias) {
  ivec2 texSize = textureSize(tex, 0);
  ivec2 pixelCoord = ivec2(round((2 * texSize.x * p.x - 1) / 2), round((2 * texSize.y * p.y - 1) / 2));

  float dx = ((2 * texSize.x * p.x - 1) / 2 - pixelCoord.x);
  float dy = ((2 * texSize.y * p.y - 1) / 2 - pixelCoord.y);

  // Get depth for each pixel in a 3 by 3 grid
  float dCenter = texelFetch(tex, pixelCoord, bias).x;
  float dUp = texelFetch(tex, pixelCoord + ivec2(0, 1), bias).x;
  float dDown = texelFetch(tex, pixelCoord + ivec2(0, -1), bias).x;
  float dLeft = texelFetch(tex, pixelCoord + ivec2(-1, 0), bias).x;
  float dRight = texelFetch(tex, pixelCoord + ivec2(1, 0), bias).x;
  float dUpLeft = texelFetch(tex, pixelCoord + ivec2(-1, 1), bias).x;
  float dDownRight = texelFetch(tex, pixelCoord + ivec2(1, -1), bias).x;
  float dDownLeft = texelFetch(tex, pixelCoord + ivec2(-1, -1), bias).x;
  float dUpRight = texelFetch(tex, pixelCoord + ivec2(1, 1), bias).x;

  int xSide = 1, ySide = 1;
  if (texSize.x != 1 && texSize.y != 1) {
    // Compute variance for each corner
    float costUpRight =
      (dUp - dCenter) * (dUp - dCenter) + (dUpRight - dCenter) * (dUpRight - dCenter) + (dRight - dCenter) * (dRight - dCenter);
    float costDownRight = (dDown - dCenter) * (dDown - dCenter) + (dDownRight - dCenter) * (dDownRight - dCenter) +
                          (dRight - dCenter) * (dRight - dCenter);
    float costUpLeft =
      (dUp - dCenter) * (dUp - dCenter) + (dUpLeft - dCenter) * (dUpLeft - dCenter) + (dLeft - dCenter) * (dLeft - dCenter);
    float costDownLeft = (dDown - dCenter) * (dDown - dCenter) + (dDownLeft - dCenter) * (dDownLeft - dCenter) +
                         (dLeft - dCenter) * (dLeft - dCenter);

    // Deal with image borders
    if (pixelCoord.x == 0) {
      costUpLeft = FLT_MAX;
      costDownLeft = FLT_MAX;
    }
    if (pixelCoord.x + 1 > texSize.x - 1) {
      costUpRight = FLT_MAX;
      costDownRight = FLT_MAX;
    }
    if (pixelCoord.y == 0) {
      costDownLeft = FLT_MAX;
      costDownRight = FLT_MAX;
    }
    if (pixelCoord.y + 1 > texSize.y - 1) {
      costUpLeft = FLT_MAX;
      costUpRight = FLT_MAX;
    }
    // Find the corner with lowest variance
    float costMin = costUpRight;
    if (costDownRight < costMin) {
      xSide = 1;
      ySide = -1;
      costMin = costDownRight;
    }
    if (costDownLeft < costMin) {
      xSide = -1;
      ySide = -1;
      costMin = costDownLeft;
    }
    if (costUpLeft < costMin) {
      xSide = -1;
      ySide = 1;
    }

    vec4 c00 = texelFetch(tex, pixelCoord, bias);
    vec4 c01 = texelFetch(tex, pixelCoord + ivec2(xSide, 0), bias);
    vec4 c10 = texelFetch(tex, pixelCoord + ivec2(0, ySide), bias);
    vec4 c11 = texelFetch(tex, pixelCoord + ivec2(xSide, ySide), bias);

    if (isinf(c00.x) || isinf(c01.x) || isinf(c10.x) || isinf(c11.x))
      return c00;
    else {
      vec4 c0 = c00 + (c01 - c00) * dx * xSide;
      vec4 c1 = c10 + (c11 - c10) * dx * xSide;

      vec4 c = c0 + (c1 - c0) * dy * ySide;

      return c;
    }
  } else if (texSize.x == 1 && texSize.y == 1) {  // No interpolation
    return texelFetch(tex, ivec2(pixelCoord.x, pixelCoord.y), bias);
  } else if (texSize.y == 1) {  // Linear interpolation
    // Pick side with smallest variance (accounting for edges)
    if ((pixelCoord.x == texSize.x - 1) || ((pixelCoord.x != 0) && (abs(dRight - dCenter) > abs(dLeft - dCenter))))
      xSide = -1;
    else
      xSide = 1;

    vec4 c0 = texelFetch(tex, pixelCoord, bias);
    vec4 c1 = texelFetch(tex, pixelCoord + ivec2(xSide, 0), bias);
    if (isinf(c0.x) || isinf(c1.x))
      return c0;
    else
      return c0 + (c1 - c0) * dx;
  } else if (texSize.x == 1) {  // Linear interpolation
    // Pick side with smallest variance (accounting for edges)
    if ((pixelCoord.y == texSize.y - 1) || ((pixelCoord.y != 0) && (abs(dUp - dCenter) > abs(dDown - dCenter))))
      ySide = -1;
    else
      ySide = 1;

    vec4 c0 = texelFetch(tex, pixelCoord, bias);
    vec4 c1 = texelFetch(tex, pixelCoord + ivec2(0, ySide), bias);
    if (isinf(c0.x) || isinf(c1.x))
      return c0;
    else
      return c0 + (c1 - c0) * dy;
  }
}

void main() {
  ivec2 texSize = textureSize(inputTextures[FRONT], 0);
  ivec2 pixelCoord = ivec2(round((texUv.x - 1 / texSize.x / 2) * texSize.x), round((texUv.y - 1 / texSize.y / 2) * texSize.y));

  vec3 coord3d;

  if (cylindrical) {
    // update the z 3D-coordinate
    float yCurrentAngle = (texUv.y - 0.5) * fovY / fovYCorrectionCoefficient + pi_2;
    coord3d = vec3(0.0, 0.0, cos(yCurrentAngle));

    // update the x spherical coordinate
    float xCurrentAngle = (0.5 - texUv.x) * fovX;

    // update the x-y 3d coordinate
    float sinY = sin(yCurrentAngle);
    coord3d.x = sinY * cos(xCurrentAngle);
    coord3d.y = sinY * sin(xCurrentAngle);
  } else {
    // Fisheye effect https://www.shadertoy.com/view/lssGD4
    vec2 d = texUv - 0.5;
    float yaw = sqrt(d.x * d.x + d.y * d.y) * fovX;

    float roll = -atan(d.y, d.x);
    float sy = -sin(yaw) * cos(roll);
    float sz = sin(yaw) * sin(roll);
    float sx = cos(yaw);
    coord3d = vec3(sx, sy, sz);
  }

  // determine on which face the 3d coordinate belongs
  float maxDot = dot(coord3d, orientations[0]);
  int face = FRONT;
  for (int i = 1; i < 6; ++i) {
    float current_dot = dot(vec3(coord3d.xy, coord3d.z / tan(min(fovY, pi_2) / 2)), orientations[i]);
    if (maxDot < current_dot) {
      maxDot = current_dot;
      face = i;
    }
  }

  // scale the 3d coordinate to be on the cube
  float absMax = 0.0;
  if (face == FRONT || face == BACK)
    absMax = abs(coord3d.x);
  else if (face == LEFT || face == RIGHT)
    absMax = abs(coord3d.y);
  else if (face == UP || face == DOWN)
    absMax = abs(coord3d.z / tan(min(fovY, pi_2) / 2));

  vec3 normalizedCoord3d = coord3d;
  if (absMax > 0.0)
    normalizedCoord3d /= absMax;

  // retrieve the x-y coordinate relatively to the current face
  // according to the 3D coordinate
  vec2 coord = vec2(0.0);
  if (face == FRONT)
    coord = normalizedCoord3d.yz;
  else if (face == RIGHT)
    coord = normalizedCoord3d.xz;
  else if (face == LEFT) {
    coord.x = -normalizedCoord3d.x;
    coord.y = normalizedCoord3d.z;
  } else if (face == UP) {
    coord.x = normalizedCoord3d.y;
    coord.y = -normalizedCoord3d.x;
  } else if (face == DOWN)
    coord = normalizedCoord3d.yx;
  else if (face == BACK) {
    coord.x = -normalizedCoord3d.y;
    coord.y = normalizedCoord3d.z;
  }

  if (face != UP && face != DOWN) {
    if (fovX < pi_2)
      coord.x /= tan(fovX / 2);
    if (fovY < pi_2)
      coord.y /= tan(fovY / 2);
  }

  vec2 faceCoord = vec2(0.5 * (1.0 - coord.x), 0.5 * (1.0 - coord.y));

  fragColor = vec4(0.0, 0.0, 0.0, 1.0);
  if (face == FRONT)
    fragColor = textureSmooth(inputTextures[0], faceCoord, 0);
  else if (face == RIGHT)
    fragColor = textureSmooth(inputTextures[1], faceCoord, 0);
  else if (face == BACK)
    fragColor = textureSmooth(inputTextures[2], faceCoord, 0);
  else if (face == LEFT)
    fragColor = textureSmooth(inputTextures[3], faceCoord, 0);
  else if (face == UP)
    fragColor = textureSmooth(inputTextures[4], faceCoord, 0);
  else if (face == DOWN)
    fragColor = textureSmooth(inputTextures[5], faceCoord, 0);

  // rectify the spherical transform
  if (rangeCamera) {
    float depth = fragColor.x;
    if (depth < maxRange) {
      float cosine = dot(coord3d, orientations[face]);
      depth = depth / cosine;
    }
    if (depth < minRange)
      depth = FLT_MAX;
    if (depth >= maxRange)
      depth = FLT_MAX;

    fragColor = vec4(depth, 0.0, 0.0, 0.0);
  }
}
