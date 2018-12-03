/*
 *   Point.cpp
 *   represents a point in 2D
 *   Author: ROBOTIS
 *
 */

#include <math.h>
#include "Point.h"

using namespace Robot;


Point2D::Point2D()
{
	X = 0;
	Y = 0;
}

Point2D::Point2D(double x, double y)
{
	X = x;
	Y = y;
}

Point2D::Point2D(const Point2D &point)
{
	X = point.X;
	Y = point.Y;
}

Point2D::~Point2D()
{
}


/*returns the euclidian distance between pt1 and pt2*/
double Point2D::Distance(Point2D &pt1, Point2D &pt2)
{
	double x = pt1.X - pt2.X;
	double y = pt1.Y - pt2.Y;
	return sqrt(x * x + y * y);
}

Point2D & Point2D::operator = (Point2D &point)
{
	X = point.X;
	Y = point.Y;
	return *this;
}

Point2D & Point2D::operator += (Point2D &point)
{
	X += point.X;
	Y += point.Y;
	return *this;
}

Point2D & Point2D::operator -= (Point2D &point)
{
	X -= point.X;
	Y -= point.Y;
	return *this;
}

Point2D & Point2D::operator += (double value)
{
	X += value;
	Y += value;
	return *this;
}

Point2D & Point2D::operator -= (double value)
{
	X -= value;
	Y -= value;
	return *this;
}

Point2D & Point2D::operator *= (double value)
{
	X *= value;
	Y *= value;
	return *this;
}

Point2D & Point2D::operator /= (double value)
{
	X /= value;
	Y /= value;
	return *this;
}

Point2D Point2D::operator + (Point2D &point)
{
	return Point2D(X + point.X, Y + point.Y);
}

Point2D Point2D::operator - (Point2D &point)
{
	return Point2D(X - point.X, Y - point.Y);
}

Point2D Point2D::operator + (double value)
{
	return Point2D(X + value, Y + value);
}

Point2D Point2D::operator - (double value)
{
	return Point2D(X - value, Y - value);
}

Point2D Point2D::operator * (double value)
{
	return Point2D(X * value, Y * value);
}

Point2D Point2D::operator / (double value)
{
	return Point2D(X / value, Y / value);
}


Point3D::Point3D()
{
	X = 0;
	Y = 0;
	Z = 0;
}

Point3D::Point3D(double x, double y, double z)
{
	X = x;
	Y = y;
	Z = z;
}

Point3D::Point3D(const Point3D &point)
{
	X = point.X;
	Y = point.Y;
	Z = point.Z;
}

Point3D::~Point3D()
{
}

double Point3D::Distance(Point3D &pt1, Point3D &pt2)
{
	double x = pt1.X - pt2.X;
	double y = pt1.Y - pt2.Y;
	double z = pt1.Z - pt2.Z;
	return sqrt(x * x + y * y + z * z);
}

Point3D & Point3D::operator = (Point3D &point)
{
	X = point.X;
	Y = point.Y;
	Z = point.Z;
	return *this;
}

Point3D & Point3D::operator += (Point3D &point)
{
	X += point.X;
	Y += point.Y;
	Z += point.Z;
	return *this;
}

Point3D & Point3D::operator -= (Point3D &point)
{
	X -= point.X;
	Y -= point.Y;
	Z -= point.Z;
	return *this;
}

Point3D & Point3D::operator += (double value)
{
	X += value;
	Y += value;
	Z += value;
	return *this;
}

Point3D & Point3D::operator -= (double value)
{
	X -= value;
	Y -= value;
	Z -= value;
	return *this;
}

Point3D & Point3D::operator *= (double value)
{
	X *= value;
	Y *= value;
	Z *= value;
	return *this;
}

Point3D & Point3D::operator /= (double value)
{
	X /= value;
	Y /= value;
	Z /= value;
	return *this;
}

Point3D Point3D::operator + (Point3D &point)
{
	return Point3D(X + point.X, Y + point.Y, Z + point.Z);
}

Point3D Point3D::operator - (Point3D &point)
{
	return Point3D(X - point.X, Y - point.Y, Z - point.Z);
}

Point3D Point3D::operator + (double value)
{
	return Point3D(X + value, Y + value, Z + value);
}

Point3D Point3D::operator - (double value)
{
	return Point3D(X - value, Y - value, Z - value);
}

Point3D Point3D::operator * (double value)
{
	return Point3D(X * value, Y * value, Z * value);
}

Point3D Point3D::operator / (double value)
{
	return Point3D(X / value, Y / value, Z / value);
}
