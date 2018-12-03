/*
 *   Vector.cpp
 *
 *   Author: ROBOTIS
 *
 */

#include <math.h>
#include "Vector.h"

using namespace Robot;


Vector3D::Vector3D()
{
	X = 0;
	Y = 0;
	Z = 0;
}

Vector3D::Vector3D(double x, double y, double z)
{
	X = x;
	Y = y;
	Z = z;
}

Vector3D::Vector3D(const Point3D &pt1, const Point3D &pt2)
{
	X = pt2.X - pt1.X;
	Y = pt2.Y - pt1.Y;
	Z = pt2.Z - pt1.Z;
}

Vector3D::Vector3D(const Vector3D &vector)
{
	X = vector.X;
	Y = vector.Y;
	Z = vector.Z;
}

Vector3D::~Vector3D()
{
}

double Vector3D::Length()
{
	return sqrt(X*X + Y*Y + Z*Z);
}

void Vector3D::Normalize()
{
	double length = Length();
	
	X = X / length;
	Y = Y / length;
	Z = Z / length;
}

double Vector3D::Dot(const Vector3D &vector)
{
	return (X*vector.X + Y*vector.Y + Z*vector.Z);
}

Vector3D Vector3D::Cross(const Vector3D &vector)
{
	Vector3D res;
	res.X = Y*vector.Z - Z*vector.Y;
	res.Y = Z*vector.X - X*vector.Z;
	res.Z = X*vector.Y - Y*vector.X;
	return res;
}

double Vector3D::AngleBetween(Vector3D &vector)
{
	return acos((X*vector.X + Y*vector.Y + Z*vector.Z) / (Length() * vector.Length())) * (180.0 / 3.141592);
}

double Vector3D::AngleBetween(Vector3D &vector, Vector3D &axis)
{
	double angle = AngleBetween(vector);
	Vector3D cross = Cross(vector);
	if(cross.Dot(axis) < 0.0)
		angle *= -1.0;

	return angle;
}

Vector3D & Vector3D::operator = (const Vector3D &vector)
{
	X = vector.X;
	Y = vector.Y;
	Z = vector.Z;
	return *this;
}

Vector3D & Vector3D::operator += (const Vector3D &vector)
{
	X += vector.X;
	Y += vector.Y;
	Z += vector.Z;
	return *this;
}

Vector3D & Vector3D::operator -= (const Vector3D &vector)
{
	X -= vector.X;
	Y -= vector.Y;
	Z -= vector.Z;
	return *this;
}

Vector3D & Vector3D::operator += (const double value)
{
	X += value;
	Y += value;
	Z += value;
	return *this;
}

Vector3D & Vector3D::operator -= (const double value)
{
	X -= value;
	Y -= value;
	Z -= value;
	return *this;
}

Vector3D & Vector3D::operator *= (const double value)
{
	X *= value;
	Y *= value;
	Z *= value;
	return *this;
}

Vector3D & Vector3D::operator /= (const double value)
{
	X /= value;
	Y /= value;
	Z /= value;
	return *this;
}

Vector3D Vector3D::operator + (const Vector3D &vector)
{
	return Vector3D(X + vector.X, Y + vector.Y, Z + vector.Z);
}

Vector3D Vector3D::operator - (const Vector3D &vector)
{
	return Vector3D(X - vector.X, Y - vector.Y, Z - vector.Z);
}

Vector3D Vector3D::operator + (const double value)
{
	return Vector3D(X + value, Y + value, Z + value);
}

Vector3D Vector3D::operator - (const double value)
{
	return Vector3D(X - value, Y - value, Z - value);
}

Vector3D Vector3D::operator * (const double value)
{
	return Vector3D(X * value, Y * value, Z * value);
}

Vector3D Vector3D::operator / (const double value)
{
	return Vector3D(X / value, Y / value, Z / value);
}
