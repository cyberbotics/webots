import WbMatrix4 from './utils/WbMatrix4.js';
import WbVector4 from './utils/WbVector4.js';
import WbTriangleMeshGeometry from './WbTriangleMeshGeometry.js';

export default class WbMesh extends WbTriangleMeshGeometry {
  constructor(id, url, ccw, name, materialIndex) {
    super(id);

    this.url = url;
    this.ccw = ccw;
    this.name = name;
    this.materialIndex = materialIndex;
    this.isCollada = this.url.endsWith('.dae');
  }

  clone(customID) {
    this.useList.push(customID);
    const clonedMesh = new WbMesh(customID, this.url, this.ccw, this.name, this.materialIndex);
    if (this.scene)
      clonedMesh.scene = this.scene;
    return clonedMesh;
  }

  _updateTriangleMesh() {
    // Assimp fix for up_axis, adapted from https://github.com/assimp/assimp/issues/849
    if (this.isCollada) { // rotate around X by 90° to swap Y and Z axis
      if (!(this.scene.rootnode.transformation instanceof WbMatrix4)) { // if it is already a WbMatrix4 it means that it is a USE node where the fix has already been applied
        let matrix = new WbMatrix4();
        matrix.set(1, 0, 0, 0, 0, 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 1);
        let rootMatrix = new WbMatrix4();
        rootMatrix.setFromArray(this.scene.rootnode.transformation);
        this.scene.rootnode.transformation = matrix.mul(rootMatrix);
      }
    }

    // create the arrays
    const coordData = [];
    const normalData = [];
    const texCoordData = [];
    const indexData = [];

    // loop over all the node to find meshes
    let queue = [];
    queue.push(this.scene.rootnode);
    let node;
    let indexOffset = 0;
    while (queue.length !== 0) {
      node = queue.shift();

      // compute absolute transform of this node from all the parents
      let transform = new WbMatrix4();
      let current = node;
      while (current) {
        let transformationMatrix;
        if (current.transformation instanceof WbMatrix4)
          transformationMatrix = current.transformation;
        else {
          transformationMatrix = new WbMatrix4();
          transformationMatrix.setFromArray(current.transformation);
        }
        transform = transform.mul(transformationMatrix);
        current = current.parent;
      }
      // merge all the meshes of this node
      for (let i = 0; i < node.meshes.length; ++i) {
        const mesh = this.scene.meshes[node.meshes[i]];
        if (this.isCollada && this.name && this.name !== mesh.name)
          continue;

        if (this.isCollada && this.materialIndex >= 0 && this.materialIndex !== mesh.materialIndex)
          continue;

        for (let j = 0; j < mesh.vertices.length / 3; j++) {
          // extract the coordinate
          const vertice = transform.mulByVec4(new WbVector4(mesh.vertices[j * 3], mesh.vertices[(j * 3) + 1], mesh.vertices[(j * 3) + 2], 1));
          coordData.push(vertice.x);
          coordData.push(vertice.y);
          coordData.push(vertice.z);
          // extract the normal
          const normal = transform.mulByVec4(new WbVector4(mesh.normals[j * 3], mesh.normals[(j * 3) + 1], mesh.normals[(j * 3) + 2], 0));
          normalData.push(normal.x);
          normalData.push(normal.y);
          normalData.push(normal.z);
          // extract the texture coordinate
          if (mesh.texturecoords && mesh.texturecoords[0]) {
            texCoordData.push(mesh.texturecoords[0][j * 2]);
            texCoordData.push(mesh.texturecoords[0][(j * 2) + 1]);
          } else {
            texCoordData.push(0.5);
            texCoordData.push(0.5);
          }
        }

        // create the index array
        for (let j = 0; j < mesh.faces.length; ++j) {
          // Skip if we do not have triangles (e.g lines)
          const face = mesh.faces[j];

          if (face.length !== 3)
            continue;

          indexData.push(face[0] + indexOffset);
          indexData.push(face[1] + indexOffset);
          indexData.push(face[2] + indexOffset);
        }

        indexOffset += mesh.vertices.length / 3;
      }

      // add all the children of this node to the queue
      if (node.children) {
        for (let i = 0; i < node.children.length; ++i)
          queue.push(node.children[i]);
      }
    }

    if (!node)
      console.warn("This file doesn't contain any mesh.");

    this._triangleMesh.initMesh(coordData, normalData, texCoordData, indexData);
  }
}
