Here are few words as how to take advantage of the Webots matrix methods when multiplying a matrix by a vector.

The point is that you can always write such a multiplication with a single instruction, avoiding thus storage or conversion.

Although matrix storage can be helpful when reusing the same matrix in several multiplications, this might help only when using the pseudo-inverse matrix or the actual inverse in Webots.
Actually, there is currently no need for such re-use optimization in all Webots code.

(1) You may want to multiply the transpose of a matrix M by a vector v. Instead of writing M.transposed() * v, just type v * M.
In the first case you compute the transpose matrix and store it in a temporary matrix, in the second case you only multiply the transpose of M by v.
See v as a column vector in the first case and as a row vector in the second.

(2) You may want to multiply the extracted 3-by-3 matrix of a 4-by-4 matrix M by a 3D-vector v. Instead of writing M.extracted3x3Matrix() * v, type M.sub3x3MatrixDot(v).

(3) You may want to multiply the transpose of the extracted 3-by-3 matrix of a 4-by-4 matrix M by a 3D-vector v. Then simply write v * M. Although the dimensions do not agree, you can naturally reinterpret either v as 4D-vector (i.e. (v, 1.0)) or M as its 3-by-3 submatrix. In both case the result is the same (identifying the 4D output (w, 1.0) with w in the first case) and it is exactly what you want.

(4) You may want to multiply a the pseudo-inverse of a 4-by-4 matrix M by a 3D-vector v. Then write M.pseudoInversed(v).

In all cases but (4), storage is useless. In (4), you may want not to recompute the translation part of the the pseudo-inverse matrix again and again and
thus set const WbMatrix4 &piM = M.pseudoInversed(). It has not been necessary so far.

Nota Bena: matrix() * v - matrix().translation() is 'equivalent' to matrix().sub3x3MatrixDot(v)
(matrix() is the WbMatrix4 of a Transform, v a WbVector3 or WbVector4) but the first expression use more operations and leads to round up errors when the translation vector is very large.
