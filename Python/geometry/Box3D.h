#ifndef MATH3D_BOX3D_H
#define MATH3D_BOX3D_H

#include "LocalCoordinates3D.h"

namespace Math3D {

struct Segment3D;
struct Plane3D;
struct Triangle3D;
struct Sphere3D;

/** @brief A 3D box
 * @ingroup Math3D
 *
 * The box is the unit cube [0,1]^3 set in the scaled local coordinate
 * system.  That is, one corner is at the origin, and it has dimensions
 * [dims.x,dims.y,dims.z] in the coordinates given by {xbasis,ybasis,zbasis}. 
 */
struct Box3D : public ScaledLocalCoordinates3D
{
  void set(const AABB3D& bb);
  void setTransformed(const AABB3D& box,const RigidTransform& T);
  void setTransformed(const Box3D& box,const RigidTransform& T);
  bool contains(const Point3D& pt) const;
  Real distance(const Point3D& pt) const;
  Real distance(const Point3D& pt,Point3D& closestPt) const;
  Real distanceSquared(const Point3D& pt,Point3D& closestPt) const;
  void getAABB(AABB3D& bb) const;
  bool intersects(const Box3D& b) const;
  bool intersectsApprox(const Box3D& b) const;  ///<faster, approximate version
  bool intersects(const Segment3D& s) const;
  bool intersects(const Line3D& l) const;
  bool intersects(const Triangle3D& t) const;
  bool intersects(const Sphere3D& b) const;
};

} //namespace Math3D

#endif
