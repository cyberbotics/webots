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
uniform int subCamerasResolutionX;
uniform int subCamerasResolutionY;

uniform float maxRange;
uniform float minRange;
uniform float fovX;
uniform float fovY;
uniform float fovYCorrectionCoefficient;

uniform sampler2D inputTextures[6];

void main() {
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
    float current_dot = dot(vec3(coord3d.xy, coord3d.z*(pi_2/fovY*fovYCorrectionCoefficient)), orientations[i]);
    if(maxDot < current_dot) {
        maxDot = current_dot;
        face = i;
    }
  }

  // scale the 3d coordinate to be on the cube
  float absMax = 0.0;
  if(face == FRONT || face == BACK)
    absMax = abs(coord3d.x);
  else if(face == LEFT || face == RIGHT)
    absMax = abs(coord3d.y);
  else if(face == UP || face == DOWN)
    absMax = abs(coord3d.z*(pi_2/fovY*fovYCorrectionCoefficient));

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
      coord.x *= pi_2 / fovX;
    if (fovY < pi_2)
      coord.y *= pi_2 / fovY * fovYCorrectionCoefficient;
  }

  vec2 faceCoord = vec2(0.5 * (1.0 - coord.x), 0.5 * (1.0 - coord.y));
//   ivec2 imageIndex = ivec2(round(faceCoord.x * (subCamerasResolutionX - 1)), round(faceCoord.y * (subCamerasResolutionY - 1)));

  fragColor = vec4(0.0, 0.0, 0.0, 1.0);
  if (face == FRONT)
    fragColor = texture(inputTextures[0], faceCoord, 0);
  else if (face == RIGHT)
    fragColor = texture(inputTextures[1], faceCoord, 0);
  else if (face == BACK)
    fragColor = texture(inputTextures[2], faceCoord, 0);
  else if (face == LEFT)
    fragColor = texture(inputTextures[3], faceCoord, 0);
  else if (face == UP)
    fragColor = texture(inputTextures[4], faceCoord, 0);
  else if (face == DOWN)
    fragColor = texture(inputTextures[5], faceCoord, 0);

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
