#ifndef ODE_INTERFACE_SIMULATOR_H
#define ODE_INTERFACE_SIMULATOR_H

#include "ODERobot.h"
#include "ODERigidObject.h"
#include "ODESurface.h"
#include "Modeling/Environment.h"
#include "Modeling/RigidObject.h"
#include <robotics/Contact.h>
#include <ode/contact.h>
#include <map>

/** @ingroup Simulation
 * @brief An index that identifies some ODE object in the world.
 * Environments, robots, robot bodies, or rigid objects are supported.
 */
struct ODEObjectID
{
  inline ODEObjectID(int _t=-1,int _i=-1,int _b=-1)
    :type(_t),index(_i),bodyIndex(_b) {}
  inline void SetEnv(int _index=0) { type = 0; index = _index; }
  inline void SetRobot(int _index=0) { type = 1; index = _index; bodyIndex = -1; }
  inline void SetRobotBody(int _index,int _bodyIndex=-1) { type = 1; index = _index; bodyIndex = _bodyIndex; }
  inline void SetRigidObject(int _index=0) { type = 2; index = _index; }
  inline bool IsEnv() const { return type == 0; }
  inline bool IsRobot() const { return type == 1; }
  inline bool IsRigidObject() const { return type == 2; }
  inline bool operator == (const ODEObjectID& rhs) const {
    if(type != rhs.type) return false;
    if(index != rhs.index) return false;
    if(type == 1 && bodyIndex!=rhs.bodyIndex) return false;
    return true;
  }
  inline bool operator < (const ODEObjectID& rhs) const {
    if(type < rhs.type) return true;
    else if(type > rhs.type) return false;
    if(index < rhs.index) return true;
    else if(index > rhs.index) return false;
    return (bodyIndex<rhs.bodyIndex);
  }

  int type;       //0: static environment, 1: robot, 2: movable object
  int index;      //index in world
  int bodyIndex;  //for robots, this identifies a link
};

/** @ingroup Simulation
 * @brief A list of contacts between two objects, returned as feedback 
 * from the simulation.
 *
 * Note: are o1 and o2 actually used?
 */
struct ODEContactList
{
  ODEObjectID o1,o2;
  //the contact points
  vector<ContactPoint> points;
  vector<Vector3> forces;

  vector<int> feedbackIndices;           //internally used
};

struct ODEContactResult;

/** @ingroup Simulation
 * @brief Global simulator settings.
 */
struct ODESimulatorSettings
{
  ODESimulatorSettings();

  double gravity[3];
  double defaultEnvPadding;
  ODESurfaceProperties defaultEnvSurface;

  //collision checking settings
  bool rigidObjectCollisions;
  bool robotSelfCollisions;
  bool robotRobotCollisions;

  //contact detection settings
  int maxContacts;
  double clusterNormalScale;;

  //ODE constants, mostly relevant to tightness of robot constraints
  double errorReductionParameter;
  double dampedLeastSquaresParameter;
};



/** @ingroup Simulation
 * @brief An interface to the ODE simulator.
 * 
 * Step() performs collision detection, calls StepDynamics(), and computes
 * collision feedback.
 *
 * StepDynamics() integrates the dynamics without setting up collision
 * detection structures.  This probably should not be used externally.
 *
 * Read/WriteState can be used to serialize state to binary.
 *
 * To get contact force information from the simulator, use the
 * EnableContactFeedback() function to initialize feedback, and then call
 * GetContactFeedback() to get a pointer to the feedback data structure.
 * Contact forces are updated after Step().
 */
class ODESimulator
{
 public:
  ODESimulator();
  virtual ~ODESimulator();
  void SetGravity(const Vector3& g);
  void SetERP(double erp);   //global error reduction  -- see ODE docs
  void SetCFM(double erp);   //global constraint force mixing -- see ODE docs
  ODESimulatorSettings& GetSettings() { return settings; }
  void AddEnvironment(const Environment& env);
  void AddRobot(Robot& robot);
  void AddObject(const RigidObject& object);
  void Step(Real dt);
  void StepDynamics(Real dt);
  bool ReadState(File& f);
  bool WriteState(File& f) const;

  size_t numEnvs() const { return envs.size(); }
  size_t numRobots() const { return robots.size(); }
  size_t numObjects() const { return objects.size(); }
  inline dWorldID world() const { return worldID; }
  const Environment* env(int i) const { return envs[i]; }
  ODETriMesh* envMesh(int i) const { return envMeshes[i]; }
  ODERobot* robot(int i) const { return robots[i]; }
  ODERigidObject* object(int i) const { return objects[i]; }
  
  void DetectCollisions();
  void ClearCollisions();
  void EnableContactFeedback(const ODEObjectID& a,const ODEObjectID& b);
  ODEContactList* GetContactFeedback(const ODEObjectID& a,const ODEObjectID& b);
  void GetContactFeedback(const ODEObjectID& a,vector<ODEContactList*>& contacts);
  bool InContact(const ODEObjectID& a) const;
  bool InContact(const ODEObjectID& a,const ODEObjectID& b) const;
  void SetupContactResponse(const ODEObjectID& a,const ODEObjectID& b,int feedbackIndex,ODEContactResult& c);
    
  //overload this to have custom parameters for surface pairs
  virtual void GetSurfaceParameters(const ODEObjectID& a,const ODEObjectID& b,dSurfaceParameters& surface) const;

 private:
  ODESimulatorSettings settings;
  dWorldID worldID;
  dSpaceID envSpaceID;
  vector<ODETriMesh*> envMeshes;
  vector<const Environment*> envs;
  vector<ODERobot*> robots;
  vector<ODERigidObject*> objects;
  map<pair<ODEObjectID,ODEObjectID>,ODEContactList> contactList;
  dJointGroupID contactGroupID;
  Real timestep;
};

#endif
