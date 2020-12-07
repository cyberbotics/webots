class WbTriangleMesh {
  constructor(){
    this.isValid;
    this.areTextureCoordinatesValid;

    this.numberOfTriangles

    this.coordinates
    this.coordIndices

    this.textureCoordinates
    this.nonRecursiveTextureCoordinates
  }

  vertex(triangle, vertex, component) {
    return this.coordinates[this.coordinateIndex(triangle, vertex, component)];
  }

  coordinateIndex(triangle, vertex, component) {
    return 3 * this.coordIndices[this.index(triangle, vertex)] + component;
  }

  index(triangle, vertex) {
     return 3 * triangle + vertex;
  }

  textureCoordinate(triangle, vertex, component) {
    return this.textureCoordinates[2 * index(triangle, vertex) + component];
  }

  nonRecursiveTextureCoordinate(triangle, vertex, component) {
    return this.nonRecursiveTextureCoordinates[2 * index(triangle, vertex) + component];
  }


  init(coord, coordIndex, normal, normalIndex, texCoord, texCoordIndex, creaseAngle, counterClockwise, normalPerVertex) {
    /*cleanup();

    mNormalPerVertex = normalPerVertex;

    // initial obvious check
    if (!coord || coord->size() == 0)
      return QString(QObject::tr("'coord' is empty."));
    if (!coordIndex || coordIndex->size() == 0)
      return QString(QObject::tr("'coordIndex' is empty."));

    const int nTrianglesEstimation = estimateNumberOfTriangles(coordIndex);  // overestimation of the number of triangles
    mNTriangles = 0;                                                         // keep the number of triangles

    // determine if the texture coordinate seems valid or not
    // this value will be used to determine the content of mTextureCoordinates
    const bool isTexCoordDefined = texCoord && texCoord->size() > 0;
    const bool isTexCoordIndexDefined = texCoordIndex && texCoordIndex->size() > 0;

    mTextureCoordinatesValid = isTexCoordDefined;
    if (isTexCoordDefined && isTexCoordIndexDefined && texCoordIndex->size() != coordIndex->size()) {
      mWarnings.append(QObject::tr("Invalid texture mapping: the sizes of 'coordIndex' and 'texCoordIndex' mismatch. The default "
                                   "texture mapping is applied."));
      mTextureCoordinatesValid = false;
    }

    // determine if the normal seems valid or not
    // this value will be used to determine the content of mNormals
    const bool isNormalDefined = normal && normal->size() > 0;
    const bool isNormalIndexDefined = normalIndex && normalIndex->size() > 0;

    mNormalsValid = isNormalDefined;
    if (mNormalPerVertex) {
      if (isNormalDefined && isNormalIndexDefined && normalIndex->size() != coordIndex->size()) {
        mWarnings.append(
          QObject::tr("Invalid normal definition: the sizes of 'coordIndex' and 'normalIndex' mismatch. The normals will "
                      "be computed using the creaseAngle."));
        mNormalsValid = false;
      }
    }

    // memory allocation of the tmp arrays (overestimated)
    const int estimateSize = 3 * nTrianglesEstimation;
    mCoordIndices.reserve(estimateSize);
    const int vertexSize = 3 * coord->size();
    if (mTextureCoordinatesValid)
      mTmpTexIndices.reserve(estimateSize);
    mTmpTriangleNormals.reserve(nTrianglesEstimation);
    mTmpVertexToTriangle.reserve(estimateSize);

    // memory allocation of the arrays (overestimated)
    mCoordinates.reserve(vertexSize);
    mTextureCoordinates.reserve(2 * estimateSize);
    mNonRecursiveTextureCoordinates.reserve(2 * estimateSize);
    mNormals.reserve(3 * estimateSize);
    mIsNormalCreased.reserve(estimateSize);

    // passes to create the final arrays
    indicesPass(coord, coordIndex, (mNormalsValid && mNormalPerVertex && isNormalIndexDefined) ? normalIndex : coordIndex,
                (isTexCoordDefined && isTexCoordIndexDefined) ? texCoordIndex : coordIndex);
    mNTriangles = mCoordIndices.size() / 3;
    if (mNormalsValid && !mNormalPerVertex && mNTriangles > normal->size()) {
      mWarnings.append(QObject::tr("Invalid normal definition: the size of 'normal' should equal the number of triangles when "
                                   "'normalPerVertex' is FALSE. The normals will be computed using the creaseAngle."));
      mNormalsValid = false;
    }
    if (!counterClockwise)
      reverseIndexOrder();
    const QString error = tmpNormalsPass(coord, normal);
    if (!error.isEmpty())
      return error;
    finalPass(coord, normal, texCoord, creaseAngle);

    // unallocate the useless data
    cleanupTmpArrays();
    mCoordinates.reserve(mCoordinates.size());
    mTextureCoordinates.reserve(mTextureCoordinates.size());
    mNonRecursiveTextureCoordinates.reserve(mNonRecursiveTextureCoordinates.size());
    mNormals.reserve(mNormals.size());
    mIsNormalCreased.reserve(mIsNormalCreased.size());

    // final obvious check
    if (mNTriangles <= 0) {
      cleanup();
      return QString(QObject::tr("The triangle mesh has no valid quad and no valid triangle."));
    }

    // validity switch
    mValid = true;

    return QString("");*/
  }

}

export {WbTriangleMesh}
