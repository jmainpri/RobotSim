#include "Utils.h"
#include <geometry/ConvexHull2D.h>
#include <statistics/KMeans.h>
using namespace Geometry;

//Produces a list of contacts as though the robot were standing on a plane.
//tol is the tolerance with which minimum-distance points are generated.
//All contacts are given zero friction and in the local frame of the robot.
void GetFlatContacts(RobotWithGeometry& robot,Real tol,ContactFormation& contacts)
{
  vector<AABB3D> bbs(robot.geometry.size());
  vector<pair<Real,int> > order;
  for(size_t i=0;i<robot.geometry.size();i++)
    if(!robot.geometry[i].verts.empty()) {
      Box3D bb;
      robot.geometry[i].UpdateTransform(robot.links[i].T_World);
      GetBB(robot.geometry[i],bb);

      AABB3D aabb;
      bb.getAABB(aabb);
      bbs[i] = aabb;
      order.push_back(pair<Real,int>(aabb.bmin.z,(int)i));
    }
  sort(order.begin(),order.end());
  Real best = Inf;
  for(size_t i=0;i<order.size();i++) {
    if(order[i].first > best) break; //done
    int k=order[i].second;
    for(size_t v=0;v<robot.geometry[k].verts.size();v++) {
      Vector3 pw = robot.links[k].T_World*robot.geometry[k].verts[v];
      if(pw.z < best)
	best = pw.z;
      assert(pw.z >= order[i].first);
    }
  }
  
  //got the plane height, now output the vertices
  ContactPoint cp;
  cp.kFriction = 0.0;
  contacts.links.resize(0);
  contacts.contacts.resize(0);
  for(size_t i=0;i<order.size();i++) {
    if(order[i].first > best+tol) break; //done
    int k=order[i].second;
    contacts.links.resize(contacts.links.size()+1);
    contacts.links.back()=k;
    contacts.contacts.resize(contacts.contacts.size()+1);
    vector<Vector3> pts;
    for(size_t v=0;v<robot.geometry[k].verts.size();v++) {
      Vector3 pw = robot.links[k].T_World*robot.geometry[k].verts[v];
      if(pw.z < best+tol) {
	pts.push_back(pw);
      }
    }
    //get the convex hull of those points
    vector<Vector2> pts2(pts.size());
    vector<Vector2> hull(pts.size());
    vector<int> hindex(pts.size());
    for(size_t v=0;v<pts.size();v++)
      pts2[v].set(pts[v].x,pts[v].y);
    int num = ConvexHull2D_Chain_Unsorted( &pts2[0], pts2.size(), &hull[0], &hindex[0]);
    contacts.contacts.back().resize(num);
    for(int v=0;v<num;v++) {
      //local estimate
      robot.links[k].T_World.mulInverse(pts[hindex[v]],cp.x);
      robot.links[k].T_World.R.mulTranspose(Vector3(0,0,1),cp.n);
      contacts.contacts.back()[v] = cp;
    }
    if(contacts.contacts.back().empty()) {
      //bound was below threshold, but no contacts below threshold
      contacts.links.resize(contacts.links.size()-1);
      contacts.contacts.resize(contacts.contacts.size()-1);
    }
  }
}

void GetFlatContacts(RobotWithGeometry& robot,int link,Real tol,vector<ContactPoint>& contacts)
{
  Real best = Inf;
  for(size_t v=0;v<robot.geometry[link].verts.size();v++) {
    Vector3 pw = robot.links[link].T_World*robot.geometry[link].verts[v];
    if(pw.z < best)
      best = pw.z;
  }
  
  //got the plane height, now output the vertices
  ContactPoint cp;
  cp.kFriction = 0.0;
  contacts.resize(0);
  vector<Vector3> pts;
  for(size_t v=0;v<robot.geometry[link].verts.size();v++) {
    Vector3 pw = robot.links[link].T_World*robot.geometry[link].verts[v];
    if(pw.z < best+tol) {
      pts.push_back(pw);
    }
  }
  //get the convex hull of those points
  vector<Vector2> pts2(pts.size());
  vector<Vector2> hull(pts.size());
  vector<int> hindex(pts.size());
  for(size_t v=0;v<pts.size();v++)
    pts2[v].set(pts[v].x,pts[v].y);
  int num = ConvexHull2D_Chain_Unsorted( &pts2[0], pts2.size(), &hull[0], &hindex[0]);
  contacts.resize(num);
  for(int v=0;v<num;v++) {
    //local estimate
    robot.links[link].T_World.mulInverse(pts[hindex[v]],cp.x);
    robot.links[link].T_World.R.mulTranspose(Vector3(0,0,1),cp.n);
    contacts[v] = cp;
  }
}

void GetFlatStance(RobotWithGeometry& robot,Real tol,Stance& s,Real kFriction)
{
  ContactFormation formation;
  GetFlatContacts(robot,tol,formation);
  LocalContactsToStance(formation,robot,s);
  for(Stance::iterator i=s.begin();i!=s.end();i++)
    for(size_t j=0;j<i->second.contacts.size();j++)
      i->second.contacts[j].kFriction = 0.25;
}

void LocalContactsToHold(const vector<ContactPoint>& contacts,int link,const RobotKinematics3D& robot,Hold& hold)
{
  hold.link = link;
  hold.contacts = contacts;
  for(size_t i=0;i<contacts.size();i++) {
    hold.contacts[i].x = robot.links[link].T_World*hold.contacts[i].x;
    hold.contacts[i].n = robot.links[link].T_World.R*hold.contacts[i].n;
  }
  MomentRotation m;
  m.setMatrix(robot.links[link].T_World.R);
  hold.SetupIKConstraint(contacts[0].x,m);
}


void LocalContactsToStance(const ContactFormation& contacts,const RobotKinematics3D& robot,Stance& stance)
{
  stance.clear();
  for(size_t i=0;i<contacts.contacts.size();i++) {
    Hold h;
    LocalContactsToHold(contacts.contacts[i],contacts.links[i],robot,h);
    stance.insert(h);
  }
}



void ClusterContacts(vector<ContactPoint>& contacts,int numClusters,Real normalScale,Real frictionScale)
{
  if((int)contacts.size() <= numClusters) return;
  vector<Vector> pts(contacts.size());
  for(size_t i=0;i<pts.size();i++) {
    pts[i].resize(7);
    pts[i][0] = contacts[i].x.x;
    pts[i][1] = contacts[i].x.y;
    pts[i][2] = contacts[i].x.z;
    pts[i][3] = contacts[i].n.x*normalScale;
    pts[i][4] = contacts[i].n.y*normalScale;
    pts[i][5] = contacts[i].n.z*normalScale;
    pts[i][6] = contacts[i].kFriction*frictionScale;
  }

  Statistics::KMeans kmeans(pts,numClusters);
  kmeans.RandomInitialCenters();
  int iters=20;
  kmeans.Iterate(iters);
  contacts.resize(kmeans.centers.size());
  vector<int> degenerate;
  for(size_t i=0;i<contacts.size();i++) {
    contacts[i].x.x = kmeans.centers[i][0];
    contacts[i].x.y = kmeans.centers[i][1];
    contacts[i].x.z = kmeans.centers[i][2];
    contacts[i].n.x = kmeans.centers[i][3];
    contacts[i].n.y = kmeans.centers[i][4];
    contacts[i].n.z = kmeans.centers[i][5];
    Real len = contacts[i].n.length();
    if(FuzzyZero(len) || !IsFinite(len)) {
      printf("ClusterContacts: Warning, clustered normal became zero/infinite\n");
      //pick any in the cluster
      int found = -1;
      for(size_t k=0;k<kmeans.labels.size();k++) {
	if(kmeans.labels[k] == (int)i) {
	  found = (int)k;
	  break;
	}
      }
      if(found < 0) {
	//strange -- degenerate cluster?
	degenerate.push_back(i);
	continue;
      }
      contacts[i].x.x = pts[found][0];
      contacts[i].x.y = pts[found][1];
      contacts[i].x.z = pts[found][2];
      contacts[i].n.x = pts[found][3];
      contacts[i].n.y = pts[found][4];
      contacts[i].n.z = pts[found][5];
      Real len = contacts[i].n.length();
      contacts[i].n /= len;
      contacts[i].kFriction = pts[found][6]/frictionScale;
      Assert(contacts[i].kFriction >= 0);
      continue;
    }
    contacts[i].n /= len;
    //cout<<"Clustered contact "<<contacts[i].pos[0]<<" "<<contacts[i].pos[1]<<" "<<contacts[i].pos[2]<<endl;
    //cout<<"Clustered normal "<<contacts[i].normal[0]<<" "<<contacts[i].normal[1]<<" "<<contacts[i].normal[2]<<endl;
    contacts[i].kFriction = kmeans.centers[i][6]/frictionScale;
    Assert(contacts[i].kFriction >= 0);
  }

  //erase backward down degenerate list 
  reverse(degenerate.begin(),degenerate.end());
  for(size_t i=0;i<degenerate.size();i++) {
    contacts.erase(contacts.begin()+degenerate[i]);
  }
}




int ClosestContact(const ContactPoint& p,const Meshing::TriMesh& mesh,ContactPoint& pclose,Real normalScale)
{
  int closest = -1;
  Real closestDist2 = Inf;
  Triangle3D tri;
  Plane3D plane;
  for(size_t i=0;i<mesh.tris.size();i++) {
    mesh.GetTriangle(i,tri);
    //first check distance to supporting plane, since it's a lower bound
    tri.getPlane(plane);
    Real dxmin = plane.distance(p.x);
    Real dn = normalScale*plane.normal.distanceSquared(p.n);
    if(dn + Sqr(dxmin) < closestDist2) {
      //has potential to be closer than previous
      Vector3 cp = tri.closestPoint(p.x);
      Real d = cp.distanceSquared(p.x) + dn;
      if(d < closestDist2) {
	closest = (int)i;
	closestDist2 = d;
	pclose.x = cp;
	pclose.n = plane.normal;
	pclose.kFriction = p.kFriction;
      }
    }
  }
  return closest;
}
