#include "Circle3D.h"
#include "geometry3d.h"
#include <stdlib.h>
using namespace std;
using namespace Math3D;


bool Circle3D::setIntersection(const Sphere3D& s,const Plane3D& p)
{
  Real d = p.distance(s.center);
  Real absd=fabs(d);
  if(absd > s.radius) return false;
  axis = p.normal;
  //projection of s.center on plane
  center = s.center - d*p.normal;
  radius = pythag_leg(absd,s.radius);
  return true;
}

Real Circle3D::distance(const Point3D& v) const
{
  //project v on plane, if it lies in disk, it's the distance to the plane
  //otherwise it's the distance to the border
  Vector3 vloc = v-center;
  Real d_axis = dot(axis,vloc);
  vloc.sub(axis,d_axis);
  //vloc is projected point
  Real d_plane = vloc.norm();
  if(d_plane <= radius) return fabs(d_axis);
  else return pythag(d_axis,d_plane-radius);
}

Real Circle3D::boundaryDistance(const Point3D& v) const
{
  Vector3 vloc = v-center;
  Real d_axis = dot(axis,vloc);
  vloc.sub(axis,d_axis);
  //vloc is projected point
  Real d_plane = vloc.norm();
  return pythag(d_axis,d_plane-radius);
}

bool Circle3D::intersects(const Circle3D& c) const
{
  cerr<<"Not done yet"<<endl;
  abort();
  return false;
}

bool Circle3D::intersects(const Sphere3D& s) const
{
  //slice the ball along the plane of the circle, check for disk intersection
  Plane3D p;
  Circle3D cs;
  getPlane(p);

  if(!cs.setIntersection(s,p)) return false;
  return Sphere3D::ballsIntersect(center,radius,cs.center,cs.radius);
}

bool Circle3D::boundaryIntersects(const Sphere3D& s) const
{
  //slice the ball along the plane of the circle, check for circle-disk intersection
  Plane3D p;
  Circle3D cs;
  getPlane(p);

  if(!cs.setIntersection(s,p)) return false;
  return Sphere3D::ballSphereIntersect(cs.center,cs.radius,center,radius);
}

bool Circle3D::intersects(const Line3D& l,Real* _t) const
{
  Plane3D p;
  getPlane(p);
  Real t;
  if(p.intersectsLine(l,&t)) {
    if(t == Inf)  { //line lies in plane
      t = l.closestPointParameter(center);
    }
    if(_t) (*_t)=t;
    Point3D pt;
    l.eval(t,pt);
    return DistanceLEQ(pt,center,radius);
  }
  return false;
}

bool Circle3D::intersects(const Plane3D& p) const
{
  Plane3D cp;
  Line3D l;
  Point3D lp;
  int res=p.allIntersections(cp,l);
  switch(res) {
  case 0: return false;
  case 1: //they intersect in line l
    l.closestPoint(center,lp);
    return DistanceLEQ(center,lp,radius);
    break;
  case 2: return true;
  default:
    cout<<"Shouldn't get here"<<endl;
    abort();
  }
  return false;
}

void Circle3D::getPlane(Plane3D& p) const
{
  p.setPointNormal(center,axis);
}

void Circle3D::getAABB(AABB3D& aabb) const
{
  aabb.setPoint(center);
  Real x,y,z;
  x = pythag_leg(axis.x,One)*radius;
  y = pythag_leg(axis.y,One)*radius;
  z = pythag_leg(axis.z,One)*radius;
  aabb.bmin.x -= x;
  aabb.bmin.y -= y;
  aabb.bmin.z -= z;
  aabb.bmax.x += x;
  aabb.bmax.y += y;
  aabb.bmax.z += z;
}
