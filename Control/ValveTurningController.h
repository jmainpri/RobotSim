#ifndef VALVETURNINGCONTROLLER_H
#define VALVETURNINGCONTROLLER_H

#ifdef OPCONTROL
#include "drchubo_controller.h"
#endif

#include "Controller.h"
#include "Modeling/DynamicPath.h"
#include <spline/PiecewisePolynomial.h>
#include <list>

class RobotWorld;


/** @brief A controller base class that reads out a desired servo position
 * and velocity using the method GetDesiredState.
 */
class ValveTurningJointController : public RobotController
{
public:
    ValveTurningJointController(Robot& robot);
    virtual ~ValveTurningJointController() {}
    virtual const char* Type() const { return "ValveTurningJointController"; }
    virtual void Update(Real dt);
    virtual void Reset();
    virtual bool ReadState(File& f) {
        if(!RobotController::ReadState(f)) return false;
        if(!qdesDefault.Read(f)) return false;
        return true;
    }
    virtual bool WriteState(File& f) const {
        if(!RobotController::WriteState(f)) return false;
        if(!qdesDefault.Write(f)) return false;
        return true;
    }
    void SetWorld( RobotWorld * w ) { world_ = w; }

    void Draw();

    //commands
    virtual vector<string> Commands() const;
    virtual bool SendCommand(const string& name,const string& str);

    ///subclasses should override this
    virtual void GetDesiredState(Config& q_des,Vector& dq_des);

#ifdef OPCONTROL
    OpSpaceControl::DRCHuboOpSpace* op_space_controller_;
#endif
    Config qdesDefault;
    Config q_des_;
    RobotWorld* world_;
    bool start_;
};

/** @ingroup Control
 * @brief A controller that uses a piecewise polynomial trajectory.
 *
 * Accepts commands set_q,append_q,set_tq,append_tq,set_qv,append_qv,brake
 */
class ValveTurningPathController : public ValveTurningJointController
{
 public:
  ValveTurningPathController(Robot& robot);
  void SetPath(const Spline::PiecewisePolynomialND& path);
  void SetPath(const vector<Config>& milestones,const vector<Real>& times);
  void SetPath(const ParabolicRamp::DynamicPath& path);
  void Append(const Spline::PiecewisePolynomialND& path);
  void Append(const ParabolicRamp::DynamicPath& path);
  void AppendLinear(const Config& config,Real dt);
  void AppendRamp(const Config& x);
  void AppendRamp(const Config& x,const Vector& v);
  void GetPath(Spline::PiecewisePolynomialND& path) const;
  void Cut(Real time,bool relative=true);
  Config Endpoint() const;
  Vector EndpointVelocity() const;
  bool Done() const;
  Real TimeRemaining() const;

  virtual const char* Type() const { return "ValveTurningPathController"; }
  virtual void GetDesiredState(Config& q_des,Vector& dq_des);
  virtual void Update(Real dt);
  virtual void Reset();
  virtual bool ReadState(File& f);
  virtual bool WriteState(File& f) const;

  //commands
  virtual vector<string> Commands() const;
  virtual bool SendCommand(const string& name,const string& str);

  Real pathOffset;
  Spline::PiecewisePolynomialND path;
};

#endif // VALVETURNINGCONTROLLER_H
