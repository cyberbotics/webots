export default class WbTriangleMesh {
  constructor() {
    this.isValid = false;

    this._coordinates = [];
    this._coordIndices = [];
    this._tmpNormalIndices = [];
    this._tmpTexIndices = [];
    this._normals = [];
    this._textureCoordinates = [];
  }

  coordinateIndex(triangle, vertex, component) {
    return 3 * this._coordIndices[this.index(triangle, vertex)] + component;
  }

  index(triangle, vertex) {
    return 3 * triangle + vertex;
  }

  init(coord, coordIndex, normal, normalIndex, texCoord, texCoordIndex) {
    this._coordIndices = coordIndex;
    this._tmpTexIndices = texCoordIndex;
    this._tmpNormalIndices = normalIndex;
    this.numberOfTriangles = this._coordIndices.length / 3;

    this._finalPass(coord, normal, texCoord);

    // validity switch
    this.isValid = true;
  }

  normal(triangle, vertex, component) {
    return this._normals[3 * this.index(triangle, vertex) + component];
  }

  textureCoordinate(triangle, vertex, component) {
    return this._textureCoordinates[2 * this.index(triangle, vertex) + component];
  }

  vertex(triangle, vertex, component) {
    return this._coordinates[this.coordinateIndex(triangle, vertex, component)];
  }

  // Private functions

  // populate this._coordinates, this._textureCoordinates and this._normals
  _finalPass(coord, normal, texCoord) {
    const texCoordSize = typeof texCoord !== 'undefined' ? texCoord.length : 0;
    const normalSize = typeof normal !== 'undefined' ? normal.length : 0;

    for (let i = 0; i < coord.length; ++i) {
      let vertex = coord[i];

      this._coordinates.push(vertex.x);
      this._coordinates.push(vertex.y);
      this._coordinates.push(vertex.z);
    }

    for (let t = 0; t < this.numberOfTriangles; ++t) { // foreach triangle
      const k = 3 * t;
      for (let v = 0; v < 3; ++v) { // foreach vertex
        const index = k + v;

        const indexNormal = this._tmpNormalIndices[index];
        if (indexNormal >= 0 && indexNormal < normalSize) {
          const nor = normal[indexNormal];
          this._normals.push(nor.x);
          this._normals.push(nor.y);
          this._normals.push(nor.z);
        }

        const indexTex = this._tmpTexIndices[index];
        if (indexTex >= 0 && indexTex < texCoordSize) {
          const tex = texCoord[indexTex];
          this._textureCoordinates.push(tex.x);
          this._textureCoordinates.push(1.0 - tex.y);
        } else {
          // foreach texture coordinate component
          this._textureCoordinates.push(0.5);
          this._textureCoordinates.push(0.5);
        }
      }
    }
  }
}
