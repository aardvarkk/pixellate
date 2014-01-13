#ifndef GEOMALGORITHMS_H
#define GEOMALGORITHMS_H

#include <cmath>

// http://geomalgorithms.com/a06-_intersect-2.html

// Copyright 2001 softSurfer, 2012 Dan Sunday
// This code may be freely used and modified for any purpose
// providing that this copyright notice is included with it.
// SoftSurfer makes no warranty for this code, and cannot be held
// liable for any real or imagined damage resulting from its use.
// Users of this code must verify correctness for their application.
 

// Assume that classes are already given for the objects:
//    Point and Vector with
//        coordinates {float x, y, z;}
//        operators for:
//            == to test  equality
//            != to test  inequality
//            (Vector)0 =  (0,0,0)         (null vector)
//            Point   = Point ± Vector
//            Vector =  Point - Point
//            Vector =  Scalar * Vector    (scalar product)
//            Vector =  Vector * Vector    (cross product)
//    Line and Ray and Segment with defining  points {Point P0, P1;}
//        (a Line is infinite, Rays and  Segments start at P0)
//        (a Ray extends beyond P1, but a  Segment ends at P1)
//    Plane with a point and a normal {Point V0; Vector  n;}
//    Triangle with defining vertices {Point V0, V1, V2;}
//    Polyline and Polygon with n vertices {int n;  Point *V;}
//        (a Polygon has V[n]=V[0])
//===================================================================
 
#define SMALL_NUM   0.00000001 // anything that avoids division overflow
// dot product (3D) which allows vector operations in arguments
#define dot(u,v)   ((u).x * (v).x + (u).y * (v).y + (u).z * (v).z)

struct Coord3D
{
  float x, y, z;

  Coord3D()
  {
    *this = Coord3D(0);
  }

  template<typename T>
  Coord3D(T x, T y, T z)
  {
    this->x = static_cast<float>(x);
    this->y = static_cast<float>(y);
    this->z = static_cast<float>(z);
  }

  template<typename T>
  Coord3D(T val)
  {
    this->x = static_cast<float>(val);
    this->y = static_cast<float>(val);
    this->z = static_cast<float>(val);
  }

  void operator=(Coord3D const& p)
  {
    this->x = p.x;
    this->y = p.y;
    this->z = p.z;
  }

  Coord3D operator*(Coord3D const& v)
  {
    return Coord3D(
      this->y * v.z - this->z * v.y,
      this->z * v.x - this->x * v.z,
      this->x * v.y - this->y * v.x);
  }

  bool operator==(Coord3D const& v) const
  {
    return (this->x == v.x) && (this->y == v.y) && (this->z == v.z);
  }

  template<typename T>
  Coord3D operator*(T scalar)
  {
    return Coord3D(this->x * scalar, this->y * scalar, this->z * scalar);
  }

  Coord3D operator+(Coord3D const& p) const
  {
    return Coord3D(this->x + p.x, this->y + p.y, this->z + p.z);
  }

  Coord3D operator-(Coord3D const& v) const
  {
    return Coord3D(this->x - v.x, this->y - v.y, this->z - v.z);
  }
};
typedef Coord3D Point;
typedef Coord3D Vector;

struct Ray
{
  Point P0, P1;
};

struct Triangle
{
  Point V0, V1, V2;
};

// intersect3D_RayTriangle(): find the 3D intersection of a ray with a triangle
//    Input:  a ray R, and a triangle T
//    Output: *I = intersection point (when it exists)
//    Return: -1 = triangle is degenerate (a segment or point)
//             0 =  disjoint (no intersect)
//             1 =  intersect in unique point I1
//             2 =  are in the same plane
int intersect3D_RayTriangle( Ray R, Triangle T, Point* I )
{
  Vector    u, v, n;              // triangle vectors
  Vector    dir, w0, w;           // ray vectors
  float     r, a, b;              // params to calc ray-plane intersect

  // get triangle edge vectors and plane normal
  u = T.V1 - T.V0;
  v = T.V2 - T.V0;
  n = u * v;              // cross product
  if (n == (Vector)0)             // triangle is degenerate
    return -1;                  // do not deal with this case

  dir = R.P1 - R.P0;              // ray direction vector
  w0 = R.P0 - T.V0;
  a = -dot(n,w0);
  b = dot(n,dir);
  if (fabs(b) < SMALL_NUM) {     // ray is  parallel to triangle plane
    if (a == 0)                 // ray lies in triangle plane
      return 2;
    else return 0;              // ray disjoint from plane
  }

  // get intersect point of ray with triangle plane
  r = a / b;
  if (r < 0.0)                    // ray goes away from triangle
    return 0;                   // => no intersect
  // for a segment, also test if (r > 1.0) => no intersect

  *I = dir * r + R.P0;            // intersect point of ray and plane

  // is I inside T?
  float    uu, uv, vv, wu, wv, D;
  uu = dot(u,u);
  uv = dot(u,v);
  vv = dot(v,v);
  w = *I - T.V0;
  wu = dot(w,u);
  wv = dot(w,v);
  D = uv * uv - uu * vv;

  // get and test parametric coords
  float s, t;
  s = (uv * wv - vv * wu) / D;
  if (s < 0.0 || s > 1.0)         // I is outside T
    return 0;
  t = (uv * wu - uu * wv) / D;
  if (t < 0.0 || (s + t) > 1.0)  // I is outside T
    return 0;

  return 1;                       // I is in T
}

#endif