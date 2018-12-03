#version 330

#define pi_2 1.570796327

// These values should coincide with the ones of WbWrenCamera::CameraOrientation
#define FRONT 0
#define RIGHT 1
#define BACK 2
#define LEFT 3
#define UP 4
#define DOWN 5

const vec3 orientations[6] = vec3[6](vec3(1.0, 0.0, 0.0), vec3(-1.0, 0.0, 0.0), vec3(0.0, 1.0, 0.0), vec3(0.0, -1.0, 0.0),
                                     vec3(0.0, 0.0, 1.0), vec3(0.0, 0.0, -1.0));

in vec2 texUv;

out vec4 fragColor;

uniform int rangeCamera;

uniform float maxRange;
uniform float minRange;
uniform float fovX;
uniform float fovY;
uniform float fovYCorrectionCoefficient;

uniform sampler2D inputTextures[6];

void main() {
  // update the z 3D-coordinate
  float yCurrentAngle = (texUv.y - 0.5) * fovY / fovYCorrectionCoefficient + pi_2;
  vec3 coord3d = vec3(0.0, 0.0, cos(yCurrentAngle));

  // update the x spherical coordinate
  float xCurrentAngle = (0.5 - texUv.x) * fovX;

  // update the x-y 3d coordinate
  float sinY = sin(yCurrentAngle);
  coord3d.x = sinY * cos(xCurrentAngle);
  coord3d.y = sinY * sin(xCurrentAngle);

  // normalize the 3d coordinate
  vec3 coord3dAbs = abs(coord3d);
  int maxIndex = 0;
  float signedValue = coord3d.x;
  float absMax = coord3dAbs.x;
  if (coord3dAbs.y > absMax) {
    maxIndex = 1;
    signedValue = coord3d.y;
    absMax = coord3dAbs.y;
  }
  if (coord3dAbs.z > absMax) {
    maxIndex = 2;
    signedValue = coord3d.z;
    absMax = coord3dAbs.z;
  }

  vec3 normalizedCoord3d = coord3d;
  if (absMax > 0.0)
    normalizedCoord3d /= absMax;

  // determine on which face the 3d coordinate hits the cube
  bool isNegative = (signedValue < 0.0);
  int face = FRONT;
  if (maxIndex == 0) {
    if (isNegative)
      face = BACK;
    else
      face = FRONT;
  } else if (maxIndex == 1) {
    if (isNegative)
      face = RIGHT;
    else
      face = LEFT;
  } else if (maxIndex == 2) {
    if (isNegative)
      face = DOWN;
    else
      face = UP;
  }

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

  if (fovX < pi_2)
    coord.x *= pi_2 / fovX;
  if (fovY < pi_2)
    coord.y *= pi_2 / fovY;

  vec2 faceCoord = vec2(0.5 * (1.0 - coord.x), 0.5 * (1.0 - coord.y));

  fragColor = vec4(0.0, 0.0, 0.0, 1.0);
  if (face == FRONT)
    fragColor = texture(inputTextures[0], faceCoord);
  else if (face == RIGHT)
    fragColor = texture(inputTextures[1], faceCoord);
  else if (face == BACK)
    fragColor = texture(inputTextures[2], faceCoord);
  else if (face == LEFT)
    fragColor = texture(inputTextures[3], faceCoord);
  else if (face == UP)
    fragColor = texture(inputTextures[4], faceCoord);
  else if (face == DOWN)
    fragColor = texture(inputTextures[5], faceCoord);

  // rectify the spherical transform
  if (rangeCamera > 0) {
    float depth = fragColor.x;
    if (depth < maxRange) {
      float cosine = 0.0f;
      for (int i = 0; i < 6; ++i) {
        float cosineTmp = dot(normalizedCoord3d, orientations[i]);
        cosineTmp = cosineTmp / length(normalizedCoord3d);
        if (cosineTmp > cosine)
          cosine = cosineTmp;
      }
      depth = depth / cosine;
    }
    if (depth < minRange)
      depth = maxRange;

    fragColor = vec4(clamp(depth, 0.0, maxRange), 0.0, 0.0, 0.0);
  }
}
