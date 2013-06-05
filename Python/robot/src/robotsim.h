#ifndef ROBOTSIM_H
#define ROBOTSIM_H

#include <stddef.h>
#include "robotmodel.h"
#include <Simulation/ODESurface.h>

class Simulator;
class SimRobotController;

//definitions for internal objects
class SensorBase;
class WorldSimulation;
class ODETriMesh;
typedef struct dxBody *dBodyID;

/** @brief A sensor on a simulated robot.  Retreive this from the controller,
 * and use getMeasurements to get the currently simulated measurement vector.
 *
 * type() gives you a string defining the sensor type.
 * measurementNames() gives you a list of names for the measurements.
 */
class SimRobotSensor
{
 public:
  SimRobotSensor(SensorBase* sensor);
  std::string name();
  std::string type();
  std::vector<std::string> measurementNames();
  void getMeasurements(std::vector<double>& out);

  SensorBase* sensor; 
};

/** @brief A controller for a simulated robot.
 *
 * The basic way of using this is in "standard" move-to mode which accepts
 * a milestone (setMilestone) or list of milestones (repeated calls to
 * addMilestone) and interpolates dynamically from the current
 * configuration/velocity.  To handle disturbances, a PID loop is run.
 * The constants of this loop are initially set in the robot file, or you can
 * perform tuning via setPIDGains.
 *
 * Arbitrary trajectories can be tracked by using setVelocity over short time
 * steps.  Force controllers can be implemented using setTorque, again using
 * short time steps.  These set the controller into manual override mode.
 * To reset back to regular motion queue control, 
 */
class SimRobotController
{
 public:
  SimRobotController();
  ~SimRobotController();
  /// Sets the current feedback control rate
  void setRate(double dt);

  /// Returns the current commanded configuration
  void getCommandedConfig(std::vector<double>& out);
  /// Returns the current commanded velocity
  void getCommandedVelocity(std::vector<double>& out);

  /// Returns the current "sensed" configuration from the simulator
  void getSensedConfig(std::vector<double>& out);
  /// Returns the current "sensed" velocity from the simulator
  void getSensedVelocity(std::vector<double>& out);

  /// Returns a sensor by index.  If out of bounds, a null sensor is returned
  SimRobotSensor getSensor(int index);
  /// Returns a sensor by name.  If unavailable, a null sensor is returned
  SimRobotSensor getNamedSensor(const std::string& name);

  /// gets a command list
  std::vector<std::string> commands();
  /// sends a command to the controller
  bool sendCommand(const std::string& name,const std::string& args);

  /// gets/sets settings of the controller
  std::string getSetting(const std::string& name);
  bool setSetting(const std::string& name,const std::string& val);

  /// Uses a dynamic interpolant to get from the current state to the
  /// desired milestone (with optional ending velocity).  This interpolant
  /// is time-optimal with respect to the velocity and acceleration bounds.
  void setMilestone(const std::vector<double>& q);
  void setMilestone(const std::vector<double>& q,const std::vector<double>& dq);
  /// Same as setMilestone, but appends an interpolant onto an internal
  /// motion queue starting at the current queued end state.
  void addMilestone(const std::vector<double>& q);
  void addMilestone(const std::vector<double>& q,const std::vector<double>& dq);

  /// Returns the remaining duration of the motion queue
  double remainingTime() const;

  /// Sets a rate controller from the current commanded config to move at
  /// rate dq for time dt.
  void setVelocity(const std::vector<double>& dq,double dt);
  /// Sets a torque command controller
  void setTorque(const std::vector<double>& t);
  /// Sets a PID command controller 
  void setPIDCommand(const std::vector<double>& qdes,const std::vector<double>& dqdes);
  /// Turns on/off manual mode, if either the setTorque or setPID command were
  /// previously set.
  void setManualMode(bool enabled);

  /// Sets the PID gains
  void setPIDGains(const std::vector<double>& kP,const std::vector<double>& kI,const std::vector<double>& kD);

  int index;
  WorldSimulation* sim;
};

/** @brief A reference to a rigid body inside a Simulator (either a
 * RigidObjectModel, TerrainModel, or a link of a RobotModel).
 */
class SimBody
{
 public:
  /// Applies a force and torque about the COM
  void applyWrench(const double f[3],const double t[3]);
  /// Sets the angular velocity and translational velocity 
  void setVelocity(const double w[3],const double v[3]);
  /// Returns the angular velocity and translational velocity 
  void getVelocity(double out[3],double out2[3]);
  void setTransform(const double R[9],double t[3]);
  void getTransform(double out[9],double out2[3]);

  /// Sets the collision padding (useful for thin objects)
  void setCollisionPadding(double padding);
  double getCollisionPadding();

  /// Gets/sets the surface properties
  ODESurfaceProperties* surface();  

  ODETriMesh* mesh;
  dBodyID body;
};

/** @brief A dynamics simulator for a WorldModel. 
 */
class Simulator
{
 public:
  /// Constructs the simulator from a WorldModel.  If the WorldModel was
  /// loaded from an XML file, then the simulation setup is loaded from it.
  Simulator(const WorldModel& model);
  ~Simulator();

  /// Resets to the initial state (same as setState(initialState))
  void reset();
  /// Returns the associated world model
  WorldModel getWorld() const;

  /// Returns a Base64 string representing the binary data for the current
  /// simulation state, including controller parameters, etc.
  std::string getState();
  /// Sets the current simulation state from a Base64 string returned by
  /// a prior getState call.
  void setState(const std::string& str);

  /// Advances the simulation by time t, and updates the world model from the
  /// simulation state.
  void simulate(double t);
  /// Advances a faked simulation by time t, and updates the world model
  /// from the faked simulation state.
  void fakeSimulate(double t);
  /// Returns the simulation time
  double getTime();

  /// Updates the world model from the current simulation state.  This only
  /// needs to be called if you change the world model and want to revert
  /// back to the simulation state.
  void updateWorld();

  /// Returns the current actual configuration of the robot from the simulator
  void getActualConfig(int robot,std::vector<double>& out);
  /// Returns the current actual velocity of the robot from the simulator
  void getActualVelocity(int robot,std::vector<double>& out);
  /// Returns the current actual torques on the robot's drivers
  /// from the simulator
  void getActualTorques(int robot,std::vector<double>& out);

  /// Call this to enable contact feedback between the two objects
  /// (arguments are indexes returned by object.getID()).  Contact feedback
  /// has a small overhead so you may want to do this selectively.
  void enableContactFeedback(int obj1,int obj2);
  /// Call this to enable contact feedback between all pairs of objects.
  /// Contact feedback has a small overhead so you may want to do this
  /// selectively.
  void enableContactFeedbackAll();
  /// Returns true if the objects (indexes returned by object.getID()) are in
  /// contact on the current time step
  bool inContact(int aid,int bid);
  /// Returns the list of contacts (x,n,kFriction) at the last time step.
  /// Normals point into object a.
  void getContacts(int aid,int bid,std::vector<std::vector<double> >& out);
  /// Returns the list of contact forces on object a at the last time step
  void getContactForces(int aid,int bid,std::vector<std::vector<double> >& out);
  /// Returns the contact force on object a at the last time step
  void contactForce(int aid,int bid,double out[3]);
  /// Returns true if the objects had contact over the last simulate() call
  bool hadContact(int aid,int bid);
  /// Returns true if the objects had ever separated during the last
  /// simulate() call
  bool hadSeparation(int aid,int bid);
  /// Returns the average contact force on object a over the last simulate()
  /// call
  void meanContactForce(int aid,int bid,double out[3]);

  /// Returns a controller for the indicated robot
  SimRobotController getController(int robot);
  SimRobotController getController(const RobotModel& robot);
  SimBody getBody(const RobotModelLink& link);
  SimBody getBody(const RigidObjectModel& object);
  SimBody getBody(const TerrainModel& terrain);

  /// Returns the joint force and torque local to the link, as would be read
  /// by a force-torque sensor mounted at the given link's origin.  The 6
  /// entries are (fx,fy,fz,mx,my,mz)
  void getJointForces(const RobotModelLink& link,double out[6]);

  /// Sets the overall gravity vector
  void setGravity(const double g[3]);
  /// Sets the internal simulation substep.  Values < 0.01 are recommended.
  void setSimStep(double dt);

  int index;
  WorldModel world;
  WorldSimulation* sim;
  std::string initialState;
};

#endif
