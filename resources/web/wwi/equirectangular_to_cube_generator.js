/* global THREE */

// Source: https://github.com/mrdoob/three.js/blob/r104/examples/js/loaders/EquirectangularToCubeGenerator.js
// @author Richard M. / https://github.com/richardmonette
// @author WestLangley / http://github.com/WestLangley

THREE.EquirectangularToCubeGenerator = (function() {
  var camera = new THREE.PerspectiveCamera(90, 1, 0.1, 10);
  var scene = new THREE.Scene();
  var boxMesh = new THREE.Mesh(new THREE.BoxBufferGeometry(1, 1, 1), getShader());
  boxMesh.material.side = THREE.BackSide;
  scene.add(boxMesh);

  class EquirectangularToCubeGenerator {
    constructor(sourceTexture, options) {
      options = options || {};

      this.sourceTexture = sourceTexture;
      this.resolution = options.resolution || 512;

      this.views = [
        { t: [ 1, 0, 0 ], u: [ 0, -1, 0 ] },
        { t: [ -1, 0, 0 ], u: [ 0, -1, 0 ] },
        { t: [ 0, 1, 0 ], u: [ 0, 0, 1 ] },
        { t: [ 0, -1, 0 ], u: [ 0, 0, -1 ] },
        { t: [ 0, 0, 1 ], u: [ 0, -1, 0 ] },
        { t: [ 0, 0, -1 ], u: [ 0, -1, 0 ] }
      ];

      var params = {
        format: options.format || this.sourceTexture.format,
        magFilter: this.sourceTexture.magFilter,
        minFilter: this.sourceTexture.minFilter,
        type: options.type || this.sourceTexture.type,
        generateMipmaps: this.sourceTexture.generateMipmaps,
        anisotropy: this.sourceTexture.anisotropy,
        encoding: this.sourceTexture.encoding
      };

      this.renderTarget = new THREE.WebGLRenderTargetCube(this.resolution, this.resolution, params);
    }

    update(renderer) {
      var currentRenderTarget = renderer.getRenderTarget();

      boxMesh.material.uniforms.equirectangularMap.value = this.sourceTexture;

      for (let i = 0; i < 6; i++) {
        var v = this.views[ i ];

        camera.position.set(0, 0, 0);
        camera.up.set(v.u[ 0 ], v.u[ 1 ], v.u[ 2 ]);
        camera.lookAt(v.t[ 0 ], v.t[ 1 ], v.t[ 2 ]);

        renderer.setRenderTarget(this.renderTarget, i);
        renderer.clear();
        renderer.render(scene, camera);
      }

      renderer.setRenderTarget(currentRenderTarget);

      return this.renderTarget.texture;
    }

    dispose() {
      this.renderTarget.dispose();
    }
  };

  function getShader() {
    var shaderMaterial = new THREE.ShaderMaterial({
      uniforms: {
        'equirectangularMap': { value: null }
      },
      vertexShader:
`varying vec3 localPosition;

void main() {
  localPosition = position;
  gl_Position = projectionMatrix * modelViewMatrix * vec4( position, 1.0 );
}`,
      fragmentShader:
`#include <common>
varying vec3 localPosition;
uniform sampler2D equirectangularMap;

#define RECIPROCAL_PI 0.31830988618
#define RECIPROCAL_PI2 0.15915494
vec2 EquirectangularSampleUV(vec3 v) {
  vec2 uv = vec2(atan(v.z, v.x), asin(v.y));
  uv *= vec2(-RECIPROCAL_PI2, RECIPROCAL_PI); // inverse atan and flipX to match Webots orientation
  uv += 0.5;
  return uv;
}

void main() {
  vec2 uv = EquirectangularSampleUV(normalize(localPosition));
  gl_FragColor = texture2D(equirectangularMap, uv);
}`,
      blending: THREE.NoBlending
    });

    shaderMaterial.type = 'EquirectangularToCubeGenerator';

    return shaderMaterial;
  }

  return EquirectangularToCubeGenerator;
})();
