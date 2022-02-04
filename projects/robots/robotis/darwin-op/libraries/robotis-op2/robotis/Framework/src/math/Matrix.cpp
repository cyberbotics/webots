/*
 *   Matrix.cpp
 *   Matrix3D is the class of matrix of size 4X4.
 *   Author: ROBOTIS
 *
 */

#include <math.h>
#include "Matrix.h"

using namespace Robot;

/**
create the identity matrix (of size 4X4)
*/
Matrix3D::Matrix3D()
{
	Identity();
}

/**
TODO: no clear about the side effects
*/
Matrix3D::Matrix3D(const Matrix3D &mat)
{
	*this = mat;
}

Matrix3D::~Matrix3D()
{
}


/*
set the current matrix to identity
*/
void Matrix3D::Identity()
{
	m[m00] = 1; m[m01] = 0; m[m02] = 0; m[m03] = 0;
	m[m10] = 0; m[m11] = 1; m[m12] = 0; m[m13] = 0;
	m[m20] = 0; m[m21] = 0; m[m22] = 1; m[m23] = 0;
	m[m30] = 0; m[m31] = 0; m[m32] = 0; m[m33] = 1;
}


/*
compute the inverse of the matrix
returns true iff the matrix is invertible
*/
bool Matrix3D::Inverse()
{
	Matrix3D src, dst, tmp;
	double det;

    /* transpose matrix */
    for (int i = 0; i < 4; i++)
    {
        src.m[i] = m[i*4];
        src.m[i + 4] = m[i*4 + 1];
        src.m[i + 8] = m[i*4 + 2];
        src.m[i + 12] = m[i*4 + 3];
    }

    /* calculate pairs for first 8 elements (cofactors) */
    tmp.m[0] = src.m[10] * src.m[15];
    tmp.m[1] = src.m[11] * src.m[14];
    tmp.m[2] = src.m[9] * src.m[15];
    tmp.m[3] = src.m[11] * src.m[13];
    tmp.m[4] = src.m[9] * src.m[14];
    tmp.m[5] = src.m[10] * src.m[13];
    tmp.m[6] = src.m[8] * src.m[15];
    tmp.m[7] = src.m[11] * src.m[12];
    tmp.m[8] = src.m[8] * src.m[14];
    tmp.m[9] = src.m[10] * src.m[12];
    tmp.m[10] = src.m[8] * src.m[13];
    tmp.m[11] = src.m[9] * src.m[12];
    /* calculate first 8 elements (cofactors) */
    dst.m[0] = (tmp.m[0]*src.m[5] + tmp.m[3]*src.m[6] + tmp.m[4]*src.m[7]) - (tmp.m[1]*src.m[5] + tmp.m[2]*src.m[6] + tmp.m[5]*src.m[7]);
    dst.m[1] = (tmp.m[1]*src.m[4] + tmp.m[6]*src.m[6] + tmp.m[9]*src.m[7]) - (tmp.m[0]*src.m[4] + tmp.m[7]*src.m[6] + tmp.m[8]*src.m[7]);
    dst.m[2] = (tmp.m[2]*src.m[4] + tmp.m[7]*src.m[5] + tmp.m[10]*src.m[7]) - (tmp.m[3]*src.m[4] + tmp.m[6]*src.m[5] + tmp.m[11]*src.m[7]);
    dst.m[3] = (tmp.m[5]*src.m[4] + tmp.m[8]*src.m[5] + tmp.m[11]*src.m[6]) - (tmp.m[4]*src.m[4] + tmp.m[9]*src.m[5] + tmp.m[10]*src.m[6]);
    dst.m[4] = (tmp.m[1]*src.m[1] + tmp.m[2]*src.m[2] + tmp.m[5]*src.m[3]) - (tmp.m[0]*src.m[1] + tmp.m[3]*src.m[2] + tmp.m[4]*src.m[3]);
    dst.m[5] = (tmp.m[0]*src.m[0] + tmp.m[7]*src.m[2] + tmp.m[8]*src.m[3]) - (tmp.m[1]*src.m[0] + tmp.m[6]*src.m[2] + tmp.m[9]*src.m[3]);
    dst.m[6] = (tmp.m[3]*src.m[0] + tmp.m[6]*src.m[1] + tmp.m[11]*src.m[3]) - (tmp.m[2]*src.m[0] + tmp.m[7]*src.m[1] + tmp.m[10]*src.m[3]);
    dst.m[7] = (tmp.m[4]*src.m[0] + tmp.m[9]*src.m[1] + tmp.m[10]*src.m[2]) - (tmp.m[5]*src.m[0] + tmp.m[8]*src.m[1] + tmp.m[11]*src.m[2]);
    /* calculate pairs for second 8 elements (cofactors) */
    tmp.m[0] = src.m[2]*src.m[7];
    tmp.m[1] = src.m[3]*src.m[6];
    tmp.m[2] = src.m[1]*src.m[7];
    tmp.m[3] = src.m[3]*src.m[5];
    tmp.m[4] = src.m[1]*src.m[6];
    tmp.m[5] = src.m[2]*src.m[5];
    //Streaming SIMD Extensions - Inverse of 4x4 Matrix 8
    tmp.m[6] = src.m[0]*src.m[7];
    tmp.m[7] = src.m[3]*src.m[4];
    tmp.m[8] = src.m[0]*src.m[6];
    tmp.m[9] = src.m[2]*src.m[4];
    tmp.m[10] = src.m[0]*src.m[5];
    tmp.m[11] = src.m[1]*src.m[4];
    /* calculate second 8 elements (cofactors) */
    dst.m[8] = (tmp.m[0]*src.m[13] + tmp.m[3]*src.m[14] + tmp.m[4]*src.m[15]) - (tmp.m[1]*src.m[13] + tmp.m[2]*src.m[14] + tmp.m[5]*src.m[15]);
    dst.m[9] = (tmp.m[1]*src.m[12] + tmp.m[6]*src.m[14] + tmp.m[9]*src.m[15]) - (tmp.m[0]*src.m[12] + tmp.m[7]*src.m[14] + tmp.m[8]*src.m[15]);
    dst.m[10] = (tmp.m[2]*src.m[12] + tmp.m[7]*src.m[13] + tmp.m[10]*src.m[15]) - (tmp.m[3]*src.m[12] + tmp.m[6]*src.m[13] + tmp.m[11]*src.m[15]);
    dst.m[11] = (tmp.m[5]*src.m[12] + tmp.m[8]*src.m[13] + tmp.m[11]*src.m[14]) - (tmp.m[4]*src.m[12] + tmp.m[9]*src.m[13] + tmp.m[10]*src.m[14]);
    dst.m[12] = (tmp.m[2]*src.m[10] + tmp.m[5]*src.m[11] + tmp.m[1]*src.m[9]) - (tmp.m[4]*src.m[11] + tmp.m[0]*src.m[9] + tmp.m[3]*src.m[10]);
    dst.m[13] = (tmp.m[8]*src.m[11] + tmp.m[0]*src.m[8] + tmp.m[7]*src.m[10]) - (tmp.m[6]*src.m[10] + tmp.m[9]*src.m[11] + tmp.m[1]*src.m[8]);
    dst.m[14] = (tmp.m[6]*src.m[9] + tmp.m[11]*src.m[11] + tmp.m[3]*src.m[8]) - (tmp.m[10]*src.m[11] + tmp.m[2]*src.m[8] + tmp.m[7]*src.m[9]);
    dst.m[15] = (tmp.m[10]*src.m[10] + tmp.m[4]*src.m[8] + tmp.m[9]*src.m[9]) - (tmp.m[8]*src.m[9] + tmp.m[11]*src.m[10] + tmp.m[5]*src.m[8]);
    /* calculate determinant */
    det = src.m[0]*dst.m[0] + src.m[1]*dst.m[1] + src.m[2]*dst.m[2] + src.m[3]*dst.m[3];
    /* calculate matrix inverse */
    if (det == 0)
    {
        det = 0;
        return false;
    }
    else
    {
        det = 1 / det;
    }

    for (int i = 0; i < MAXNUM_ELEMENT; i++)
        m[i] = dst.m[i] * det;

    return true;
}


/*
compute the matrix
scale.X         0        0        0
0            scale.Y     0        0
0               0      scale.Z    0
0               0        0        1
*/
void Matrix3D::Scale(Vector3D scale)
{
	Matrix3D mat;
	mat.m[m00] = scale.X;
	mat.m[m11] = scale.Y;
	mat.m[m22] = scale.Z;

	*this *= mat;
}



/*construct the matrix corresponding to a rotation of angle around the axis*/
void Matrix3D::Rotate(double angle, Vector3D axis)
{
	double rad = angle * 3.141592 / 180.0;
	double C = cos(rad);
	double S = sin(rad);
	Matrix3D mat;

	mat.m[m00] = C + axis.X * axis.X * (1 - C);
	mat.m[m01] = axis.X * axis.Y * (1 - C) - axis.Z * S;
	mat.m[m02] = axis.X * axis.Z * (1 - C) + axis.Y * S;
	mat.m[m10] = axis.X * axis.Y * (1 - C) + axis.Z * S;
	mat.m[m11] = C + axis.Y * axis.Y * (1 - C);
	mat.m[m12] = axis.Y * axis.Z * (1 - C) - axis.X * S;
	mat.m[m20] = axis.X * axis.Z * (1 - C) - axis.Y * S;
	mat.m[m21] = axis.Y * axis.Z * (1 - C) + axis.X * S;
	mat.m[m22] = C + axis.Z * axis.Z * (1 - C);

	*this *= mat;
}


/*compute the matrix corresponding to a translation of vector offset*/
void Matrix3D::Translate(Vector3D offset)
{
	Matrix3D mat;
	mat.m[m03] = offset.X;
	mat.m[m13] = offset.Y;
	mat.m[m23] = offset.Z;

	*this *= mat;
}


/*make the product of the current matrix with the point*/
Point3D Matrix3D::Transform(Point3D point)
{
	Point3D result;
	result.X = m[m00]*point.X + m[m01]*point.Y + m[m02]*point.Z + m[m03];
	result.Y = m[m10]*point.X + m[m11]*point.Y + m[m12]*point.Z + m[m13];
	result.Z = m[m20]*point.X + m[m21]*point.Y + m[m22]*point.Z + m[m23];

    return result;
}


/*make the product of the current matrix with the point*/
Vector3D Matrix3D::Transform(Vector3D vector)
{
	Vector3D result;
	result.X = m[m00]*vector.X + m[m01]*vector.Y + m[m02]*vector.Z + m[m03];
	result.Y = m[m10]*vector.X + m[m11]*vector.Y + m[m12]*vector.Z + m[m13];
	result.Z = m[m20]*vector.X + m[m21]*vector.Y + m[m22]*vector.Z + m[m23];

    return result;
}

/*?*/
void Matrix3D::SetTransform(Point3D point, Vector3D angle)
{
	double Cx = cos(angle.X * 3.141592 / 180.0);
	double Cy = cos(angle.Y * 3.141592 / 180.0);
	double Cz = cos(angle.Z * 3.141592 / 180.0);
	double Sx = sin(angle.X * 3.141592 / 180.0);
	double Sy = sin(angle.Y * 3.141592 / 180.0);
	double Sz = sin(angle.Z * 3.141592 / 180.0);
	
	Identity();
	m[0] = Cz * Cy;
    m[1] = Cz * Sy * Sx - Sz * Cx;
    m[2] = Cz * Sy * Cx + Sz * Sx;
	m[3] = point.X;
    m[4] = Sz * Cy;
    m[5] = Sz * Sy * Sx + Cz * Cx;
    m[6] = Sz * Sy * Cx - Cz * Sx;
    m[7] = point.Y;
    m[8] = -Sy;
    m[9] = Cy * Sx;
    m[10] = Cy * Cx;
    m[11] = point.Z;
}

Matrix3D & Matrix3D::operator = (const Matrix3D &mat)
{
	for (int i = 0; i < MAXNUM_ELEMENT; i++)
        m[i] = mat.m[i];
	return *this;
}

Matrix3D & Matrix3D::operator *= (const Matrix3D &mat)
{
	Matrix3D tmp = *this * mat;
	*this = tmp;
	return *this;
}

Matrix3D Matrix3D::operator * (const Matrix3D &mat)
{
	Matrix3D result;

    for (int j = 0; j < 4; j++)
    {
        for (int i = 0; i < 4; i++)
        {
            for (int c = 0; c < 4; c++)
            {
				result.m[j*4+i] += m[j*4+c] * mat.m[c*4+i];
            }
        }
    }

	return result;
}
