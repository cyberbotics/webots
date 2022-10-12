export default class WbTriangleMesh {
  #coordinates;
  #coordIndices;
  #normals;
  #textureCoordinates;
  #tmpNormalIndices;
  #tmpTexIndices;
  constructor() {
    this.isValid = false;

    this.#coordinates = [];
    this.#coordIndices = [];
    this.#tmpNormalIndices = [];
    this.#tmpTexIndices = [];
    this.#normals = [];
    this.#textureCoordinates = [];
  }

  coordinateIndex(triangle, vertex, component) {
    return 3 * this.#coordIndices[this.index(triangle, vertex)] + component;
  }

  index(triangle, vertex) {
    return 3 * triangle + vertex;
  }

  init(coord, coordIndex, normal, normalIndex, texCoord, texCoordIndex) {
    this.#coordIndices = coordIndex;
    this.#tmpTexIndices = texCoordIndex;
    this.#tmpNormalIndices = normalIndex;
    this.numberOfTriangles = this.#coordIndices.length / 3;

    this.#finalPass(coord, normal, texCoord);

    // validity switch
    this.isValid = true;
  }

  initMesh(coord, normal, texCoord, index) {
    this.numberOfTriangles = index.length / 3;

    this.#coordIndices = [...index];
    this.#coordinates = [...coord];

    for (let t = 0; t < this.numberOfTriangles; ++t) { // foreach triangle
      for (let v = 0; v < 3; ++v) { // foreach vertex
        const currentIndex = this.#coordIndices[3 * t + v];
        this.#textureCoordinates.push(texCoord[2 * currentIndex]);
        this.#textureCoordinates.push(texCoord[2 * currentIndex + 1]);
        this.#normals.push(normal[3 * currentIndex]);
        this.#normals.push(normal[3 * currentIndex + 1]);
        this.#normals.push(normal[3 * currentIndex + 2]);
      }
    }

    // validity switch
    this.isValid = true;
  }

  normal(triangle, vertex, component) {
    return this.#normals[3 * this.index(triangle, vertex) + component];
  }

  textureCoordinate(triangle, vertex, component) {
    return this.#textureCoordinates[2 * this.index(triangle, vertex) + component];
  }

  vertex(triangle, vertex, component) {
    return this.#coordinates[this.coordinateIndex(triangle, vertex, component)];
  }

  // Private functions

  // populate this.#coordinates, this.#textureCoordinates and this.#normals
  #finalPass(coord, normal, texCoord) {
    const texCoordSize = typeof texCoord !== 'undefined' ? texCoord.length : 0;
    const normalSize = typeof normal !== 'undefined' ? normal.length : 0;

    for (let i = 0; i < coord.length; ++i) {
      let vertex = coord[i];

      this.#coordinates.push(vertex.x);
      this.#coordinates.push(vertex.y);
      this.#coordinates.push(vertex.z);
    }

    for (let t = 0; t < this.numberOfTriangles; ++t) { // foreach triangle
      const k = 3 * t;
      for (let v = 0; v < 3; ++v) { // foreach vertex
        const index = k + v;

        const indexNormal = this.#tmpNormalIndices[index];
        if (indexNormal >= 0 && indexNormal < normalSize) {
          const nor = normal[indexNormal];
          this.#normals.push(nor.x);
          this.#normals.push(nor.y);
          this.#normals.push(nor.z);
        }

        const indexTex = this.#tmpTexIndices[index];
        if (indexTex >= 0 && indexTex < texCoordSize) {
          const tex = texCoord[indexTex];
          this.#textureCoordinates.push(tex.x);
          this.#textureCoordinates.push(1.0 - tex.y);
        } else {
          // foreach texture coordinate component
          this.#textureCoordinates.push(0.5);
          this.#textureCoordinates.push(0.5);
        }
      }
    }
  }
}
