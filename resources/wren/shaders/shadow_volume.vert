#version 330 core

invariant gl_Position;  // On low-end GPUs, position may slightly differ causing z-fighting issues between rendering passes.

// These constants must be kept in sync with the values in Constants.hpp
const int maxDirectionalLights = 48;
const int maxPointLights = 48;
const int maxSpotLights = 48;

layout(location = 0) in vec4 vCoord;

uniform mat4 modelTransform;

struct DirectionalLight {
  vec4 colorAndIntensity;
  vec4 direction;
};

struct PointLight {
  vec4 colorAndIntensity;
  vec4 position;
  vec4 attenuationAndRadius;
};

struct SpotLight {
  vec4 colorAndIntensity;
  vec4 position;
  vec4 direction;
  vec4 attenuationAndRadius;
  vec4 spotParams;  // x: innerCutOffAngle, y: outerCutOffAngle, z: inverse range, w: unused
};

// List of active lights and ambient intensity for this frame
layout(std140) uniform Lights {
  DirectionalLight directionalLights[maxDirectionalLights];
  PointLight pointLights[maxPointLights];
  SpotLight spotLights[maxSpotLights];
  vec4 ambientLight;
}
lights;

// Index of active light (< 0 if inactive)
layout(std140) uniform LightRenderable {
  ivec4 activeLights;  // x: directional, y: point, z: spot
}
lightRenderable;

// Camera transforms for this frame
layout(std140) uniform CameraTransforms {
  mat4 view;
  mat4 projection;
  mat4 infiniteProjection;
}
cameraTransforms;

void extrude(inout vec4 coord, in vec4 lightDirection, in float w) {
  if (w == 0.0)
    coord = vec4(lightDirection.xyz, 0.0);
}

void main() {
  mat4 modelView = cameraTransforms.view * modelTransform;
  vec4 vCoordTransformed = modelView * vec4(vCoord.xyz, 1.0);

  // directional light
  if (lightRenderable.activeLights.x >= 0) {
    DirectionalLight light = lights.directionalLights[lightRenderable.activeLights.x];
    extrude(vCoordTransformed, light.direction, vCoord.w);
  } else {
    vec3 lightPosition;
    if (lightRenderable.activeLights.y >= 0)
      lightPosition = lights.pointLights[lightRenderable.activeLights.y].position.xyz;
    else
      lightPosition = lights.spotLights[lightRenderable.activeLights.z].position.xyz;

    vec4 lightDirection = vec4(vCoordTransformed.xyz - lightPosition, 0.0);
    extrude(vCoordTransformed, lightDirection, vCoord.w);
  }

  gl_Position = cameraTransforms.infiniteProjection * vCoordTransformed;
}
