#ifndef MATH3D_SPHERE3D_H
#define MATH3D_SPHERE3D_H

#include "Point.h"

namespace Math3D {

struct Line3D;
struct Segment3D;
struct Plane3D;
struct AABB3D;

/** @ingroup Math3D
 * @brief A 3D sphere class
 *
 * Represented by a center and a radius.
 * 
 * Most methods consider the sphere as the solid ball.
 * Methods that use the sphere boundary have the prefix "boundary".
 */
struct Sphere3D
{
  Real distance(const Point3D& v) const;
  bool contains(const Point3D& v) const;
  bool contains(const Sphere3D& s) const;
  bool withinDistance(const Point3D& v, Real dist) const;
  bool boundaryWithinDistance(const Point3D& v, Real dist) const;
  bool intersects(const Line3D&, Real* t1=NULL, Real* t2=NULL) const;
  bool intersects(const Segment3D&, Real* t1=NULL, Real* t2=NULL) const;
  bool intersects(const Plane3D& p) const;
  bool intersects(const Sphere3D& s) const;
  bool boundaryIntersects(const Sphere3D& s) const;
  bool boundaryIntersectsBoundary(const Sphere3D& s) const;

  void getAABB(AABB3D& bb) const;
  bool intersects(const AABB3D& bb) const;
  
  static bool ballsIntersect(const Point3D& ca,Real ra,const Point3D& cb,Real rb);
  static bool ballSphereIntersect(const Point3D& ca,Real ra,const Point3D& cb,Real rb);
  static bool spheresIntersect(const Point3D& ca,Real ra,const Point3D& cb,Real rb);

  Point3D center;
  Real radius;
};

} //namespace Math3D

#endif
