#ifndef OPERATIONALSPACETASKS_H
#define OPERATIONALSPACETASKS_H

#include <string>
#include <eigen3/Eigen/Core> // TODO include eigen
#include <eigen3/Eigen/Geometry>
#include <RobotKin/Robot.h>

/*
A base class for an operational space task. x=f(q)
Subclasses should override getters for sensed x, sensed error
(optional), sensed dx (optional), and appropriate calculation
of Jacobian.
Subclasses inherits setters for xdes, dxdes, gains, priority level,
weight, and task name.
If the task space is non-cartesian, the taskDifference method should
be overridden.
*/

class OperationalSpaceTask
{
public:
    OperationalSpaceTask();

    //! Returns task xdes that has been set
    Eigen::VectorXd GetDesiredValue()
    {
        return _xdes;
    }

    //! User calls this to set task state xdes
    Eigen::VectorXd SetDesiredValue(Eigen::VectorXd xdes)
    {
        _xdes = xdes;
    }

    //! User calls this to set task state xdes
    Eigen::VectorXd SetDesiredVelocity(Eigen::VectorXd dxdes)
    {
        _dxdes = dxdes;
    }

    //! User calls this to set PID gains for feedback control in operational space
    void SetGains(double hP=-1,double hD=-0.1,double hI=-0.1)
    {
        _hP = hP;
        _hD = hD;
        _hI = hI;
    }

    //! User calls this to set priority level. A smaller value means more important
    void SetPriority(int level=1)
    {
        _level = level;
    }

    //! User calls this to set task weight to differentiate from others on the same
    //! priority level. A larger weight means more important.
    void SetWeight(int weight)
    {
        _weight = weight;
    }

    //! Task name can be used to retrieve a task in an OperationalSpaceController instance
    void SetName(std::string name)
    {
        _name = name;
    }

    void ResetITerm(Eigen::VectorXd eI)
    {
        _eI = eI; // Change
    }

    //! Called at beginning of new timestep.
    //! Optionally does something before computing stuff in getCommandVelocity/advance. e.g., compute cached values
    void UpdateState( Eigen::VectorXd q,Eigen::VectorXd dq, double dt ) { }

    //! Get Command Velocity
    Eigen::VectorXd GetCommandVelocity( Eigen::VectorXd q, Eigen::VectorXd dq, double dt);

    //! Updates internal state: accumulates iterm and updates x_last
    void Advance(Eigen::VectorXd q, Eigen::VectorXd dq, double dt);

    //! Gets task x from sensed configuration q
    virtual Eigen::VectorXd GetSensedValue(Eigen::VectorXd dq) = 0;

    //! Gets Jacobian dx/dq(q)
    //! Subclasses MUST override this.
    virtual Eigen::VectorXd GetJacobian(Eigen::VectorXd q) = 0;

    //! Returns x(q)-xdes where - is the task-space differencing operator
    Eigen::VectorXd GetSensedError(Eigen::VectorXd q);

    //! Default: assumes a Cartesian space
    Eigen::VectorXd TaskDifference(Eigen::VectorXd a, Eigen::VectorXd b);

    //! GetSensedVelocity
    //! Gets task velocity from sensed configuration q.
    Eigen::VectorXd GetSensedVelocity(Eigen::VectorXd q, Eigen::VectorXd dq, double dt);

    //! Optionally can be overridden to visualize the task in OpenGL.
    void DrawGL(Eigen::VectorXd q);

protected:
    int _level;
    int _weight;
    std::string _name;
    double _hP;
    double _hD;
    double _hI;
    Eigen::VectorXd _qLast;
    Eigen::VectorXd _xdes;
    Eigen::VectorXd _dxdes;
    Eigen::VectorXd _eI;
};

//! Center of Mass position task subclass
class COMTask : public Task
{
    COMTask();

    double GetMass(self);

    //! Returns CoM position
    Eigen::Vector GetSensedValue(Eigen::VectorXd q);

    //! Returns axis-weighted CoM Jacobian by averaging
    Eigen::Vector getJacobian(Eigen::VectorXd q);

    void DrawGL(Eigen::VectorXd q);

private:
    double _mass;
    std::string _name;
    int baseLinkNo = baseLinkNo;
};

//! Link position/orientation task subclass.
//! Supports both absolute and relative positioning.
class LinkTask : public Task
{
    LinkTask();
    Eigen::VectorXd GetSensedValue( Eigen::VectorXd q );
    Eigen::VectorXd TaskDifference( Eigen::VectorXd a, Eigen::VectorXd b);
    Eigen::VectorXd GetJacobian( Eigen::VectorXd q );
    Eigen::VectorXd DrawGL(Eigen::VectorXd q);

private:

    int _linkNo;
    int _baseLinkNo;
    Robot _robot;
//    self.hP = -1
//    self.hD = 0
//    self.hI = 0
    Eigen::VectorXd _localPosition;
    std::string _taskType;
    std::string _name = "Link";
};

/*
class JointTask(Task):
    """A joint angle task class
    """
    def __init__(self, robot, jointIndices):
        Task.__init__(self)
        self.robot = robot
        self.jointIndices = jointIndices
        self.name = "Joint"
        pass

    def getSensedValue(self, q):
        return [q[jointi] for jointi in self.jointIndices]

    def getJacobian(self, q):
        J = []
        for jointi in self.jointIndices:
            Ji = [0] * self.robot.numLinks()
            Ji[jointi] = 1
            J.append(Ji)
        return J

class JointLimitTask(Task):
    def __init__(self,robot):
        Task.__init__(self)
        self.robot = robot
        self.buffersize = 2.0
        self.qmin,self.qmax = robot.getJointLimits()
        self.accelMax = robot.getAccelerationLimits()
        self.wscale = 0.1
        self.maxw = 10
        self.active = []
        self.weight = 0
        self.xdes = []
        self.dxdes = []
        self.name = "Joint limits"
        self.setGains(-0.1,-2.0,0)


    def updateState(self, q, dq, dt):
        """ check (q, dq) against joint limits
        Activates joint limit constraint, i.e., add a joint task
        to avoid reaching limit, when surpassing a threshold.

        Or, increase weight on this joint task as joint gets closer to its limit.

        """
        self.active = []
        self.weight = []
        self.xdes = []
        self.dxdes = []
        buffersize = self.buffersize
        wscale = self.wscale
        maxw = self.maxw
        for i,(j,dj,jmin,jmax,amax) in enumerate(zip(q,dq,self.qmin,self.qmax,self.accelMax)):
            if jmax <= jmin: continue
            jstop = j
            a = amax / buffersize
            w = 0
            ades = 0
            if dj > 0.0:
                t = dj / a
                    #j + t*dj - t^2*a/2
                jstop = j + t*dj - t*t*a*0.5
                if jstop > jmax:
                    #add task to slow down
                        #perfect accel solves for:
                        #j+ dj^2 / 2a  = jmax
                        #dj^2 / 2(jmax-j)   = a
                    if j >= jmax:
                        print "Joint",self.robot.getLink(i).getName(),"exceeded max",j,">=",jmax
                        ades = -amax
                        w = maxw
                    else:
                        alim = dj*dj/(jmax-j)*0.5
                        if alim > amax:
                            ades = -amax
                            w = maxw
                        else:
                            ades = -alim
                            w = wscale*(alim-a)/(amax-alim)
                        #print "Joint",self.robot.getLink(i).getName(),j,dj,"near upper limit",jmax,", desired accel:",ades," weight",w
            else:
                t = -dj / a
                    #j + t*dj + t^2*a/2
                jstop = j + t*dj + t*t*a*0.5
                if jstop < jmin:
                    #add task to slow down
                        #perfect accel solves for:
                        #j - dj^2 / 2a  = jmin
                        #dj^2 / 2(j-jmin)   = a
                    if j <= jmin:
                        print "Joint",self.robot.getLink(i).getName(),"exceeded min",j,"<=",jmin
                        ades = amax
                        w = maxw
                    else:
                        alim = dj*dj/(j-jmin)*0.5
                        if alim > amax:
                            ades = amax
                            w = maxw
                        else:
                            ades = alim
                            w = wscale*(alim-a)/(amax-alim)
                        #print "Joint",self.robot.getLink(i).getName(),j,dj,"near lower limit",jmin,", desired accel:",ades," weight",w
            if w > maxw:
                w = maxw
            self.active.append(i)
            self.xdes.append(max(jmin,min(jmax,j)))
            self.dxdes.append(dj+dt*ades)
            self.weight.append(w)
        if len(self.weight)==0:
            self.weight = 0
        return

    def getSensedValue(self, q):
        return [q[jointi] for jointi in self.active]

    def getJacobian(self, q):
        J = []
        for jointi in self.active:
            Ji = [0] * self.robot.numLinks()
            Ji[jointi] = 1
            J.append(Ji)
        return J
*/

#endif // OPERATIONALSPACETASKS_H
