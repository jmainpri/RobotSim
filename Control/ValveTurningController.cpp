#include "ValveTurningController.h"
#include "Modeling/DynamicPath.h"
#include "Modeling/Conversions.h"
#include <sstream>

int GetLinkIdByName(const Robot& robot, const std::string& name)
{
    for(int i=0; i<int(robot.links.size());i++)
    {
        if( robot.linkNames[i] == name )
            return i;
    }

    cout << "Error getting link by name" << endl;
    return 0;
}

const RobotLink3D& GetLinkByName(const Robot& robot, const std::string& name)
{
    return robot.links[GetLinkIdByName(robot,name)];
}

//-------------------------------------------------------
//-------------------------------------------------------
//-------------------------------------------------------

ValveTurningJointController::ValveTurningJointController(Robot& _robot) : RobotController(_robot) /*, op_space_controller_()*/
{
    qdesDefault = _robot.q;

    op_space_controller_ = new op_space_control::DRCHuboOpSpace();
    op_space_controller_->SetRobot( &_robot );
    op_space_controller_->SetLinkNames( _robot.linkNames );
    op_space_controller_->SetRobotNbDofs( _robot.q.n );

    start_ = true;

    // Uncomment to print joint mapping
    for(int i=0; i<int(robot.links.size());i++){
        cout << i << " : " << robot.LinkName(i) << endl;
    }
}

//subclasses should override this
void ValveTurningJointController::GetDesiredState( Config& q_des,Vector& dq_des )
{
    q_des = qdesDefault;
    dq_des.setZero();
}

void ValveTurningJointController::Update(Real dt)
{
    Assert(command != NULL);

    Config qdes(robot.links.size());
    Config dqdes(robot.links.size());
    GetDesiredState( qdes, dqdes );
    robot.NormalizeAngles(qdes);

//    qdes = Config(robot.links.size(),0.0);
//    dqdes = Config(robot.links.size(),0.0);

//    qdes[GetLinkIdByName(robot,"Body_LEP")] = -0.3;
//    qdes[GetLinkIdByName(robot,"Body_REP")] = -0.3;

    if( start_ )
    {
        op_space_controller_->CreateTasks( qdes, dt );
        start_ = false;
        cout << GetLinkByName(robot,"Body_TSY").T_World << endl;
    }

    // Get current state
    bool not_use_opspace = false;
    if( not_use_opspace || !sensors->GetTypedSensor<JointPositionSensor>() ) {
        if( !not_use_opspace )
            cout << "No joint positions, OpSpace disabled" << endl;
    }
    else
    {
        robot.UpdateConfig( sensors->GetTypedSensor<JointPositionSensor>()->q );

        if(!sensors->GetTypedSensor<JointVelocitySensor>())
        {
            cout << "No velocity" << endl;
            robot.dq.setZero();
        }
        else {
            robot.dq = sensors->GetTypedSensor<JointVelocitySensor>()->dq;
        }

        // Get configuration and velocity
        Config q_cur = robot.q;
        Vector dq_cur = robot.dq;

// Error builds up because of sensor error
//        qdes = q_cur;
//        qdes.madd( dqdes, dt );

        cout << "1 RAP : " << qdes[GetLinkIdByName(robot,"Body_RAP")] << endl;
        cout << "1 LAP : " << qdes[GetLinkIdByName(robot,"Body_LAP")] << endl;

        // Run OpSpace controller
        std::pair< std::vector<double> , std::vector<double> > q_opspace_;
        q_opspace_ = op_space_controller_->Trigger( q_cur, dq_cur, qdes, dqdes, dt );
        qdes    = q_opspace_.first; // modifed configuration
        dqdes   = q_opspace_.second; // modified velocity

        qdes[GetLinkIdByName(robot,"Body_RAP")] = 0.0;
        qdes[GetLinkIdByName(robot,"Body_LAP")] = 0.0;

        dqdes[GetLinkIdByName(robot,"Body_RAP")] = 0.0;
        dqdes[GetLinkIdByName(robot,"Body_LAP")] = 0.0;

        cout << "2 RAP : " << qdes[GetLinkIdByName(robot,"Body_RAP")] << endl;
        cout << "2 LAP : " << qdes[GetLinkIdByName(robot,"Body_LAP")] << endl;
    }

    //cout << "qdes : " << qdes << endl;

    for( size_t i=0; i<robot.drivers.size(); i++ )
    {
        if( robot.drivers[i].type == RobotJointDriver::Normal )
        {
            command->actuators[i].SetPID(qdes(robot.drivers[i].linkIndices[0]),  dqdes(robot.drivers[i].linkIndices[0]), command->actuators[i].iterm);
        }
        else
        {
            robot.q = qdes;
            robot.dq = dqdes;
            cout << "Desired affine driver value " << robot.GetDriverValue(i) <<", " << robot.GetDriverVelocity(i) << endl;
            command->actuators[i].SetPID( robot.GetDriverValue(i), robot.GetDriverVelocity(i), command->actuators[i].iterm );
        }
    }
    RobotController::Update(dt);
}

void ValveTurningJointController::Reset()
{
    RobotController::Reset();
}

vector<string> ValveTurningJointController::Commands() const
{
    vector<string> res;
    res.push_back("set_q");
    return res;
}

bool ValveTurningJointController::SendCommand(const string& name,const string& str)
{
    if(name == "set_q") {
        stringstream ss(str);
        ss>>qdesDefault;
        return true;
    }
    return false;
}

//-------------------------------------------------------
//-------------------------------------------------------
//-------------------------------------------------------

ValveTurningPathController::ValveTurningPathController(Robot& robot) : ValveTurningJointController(robot)
{
    pathOffset = 0;
    path.elements.resize(robot.q.n);
    for(int i=0;i<robot.q.n;i++)
        path.elements[i] = Spline::Constant(robot.q(i),0,0);
}

void ValveTurningPathController::SetPath(const Spline::PiecewisePolynomialND& _path)
{
    path = _path;
    pathOffset = 0;
}

void ValveTurningPathController::SetPath(const vector<Config>& milestones,const vector<Real>& times)
{
    vector<double> elems(milestones.size());
    for(size_t i=0;i<path.elements.size();i++) {
        for(size_t j=0;j<milestones.size();j++)
            elems[j] = milestones[j](i);
        path.elements[i] = Spline::PiecewiseLinear(elems,times);
    }
    pathOffset = 0;
}

void ValveTurningPathController::SetPath(const ParabolicRamp::DynamicPath& _path)
{
    path = Cast(_path);
    pathOffset = 0;
}

void ValveTurningPathController::Append(const Spline::PiecewisePolynomialND& _path)
{
    path.Concat(_path,true);
}

void ValveTurningPathController::Append(const ParabolicRamp::DynamicPath& _path)
{
    path.Concat(Cast(_path),true);
}

void ValveTurningPathController::AppendLinear(const Config& config,Real dt)
{
    if(dt == 0 && config != Endpoint()) {
        //want a continuous jump?
        printf("ValveTurningPathController::AppendLinear: Warning, discontinuous jump requested\n");
        path.Concat(Spline::Linear(config,config,0,0),true);
    }
    else
        path.Concat(Spline::Linear(Endpoint(),config,0,dt),true);
}

void ValveTurningPathController::AppendRamp(const Config& x)
{
    Vector zero(x.n,Zero);
    AppendRamp(x,zero);
}

void ValveTurningPathController::AppendRamp(const Config& x,const Vector& v)
{
    vector<ParabolicRamp::Vector> milestones(2);
    vector<ParabolicRamp::Vector> dmilestones(2);
    milestones[0] = Endpoint();
    milestones[1] = x;
    dmilestones[0] = EndpointVelocity();
    dmilestones[1] = v;
    ParabolicRamp::DynamicPath dpath;
    dpath.Init(robot.velMax,robot.accMax);
    dpath.SetJointLimits(robot.qMin,robot.qMax);
    dpath.SetMilestones(milestones,dmilestones);
    path.Concat(Cast(dpath),true);
}

void ValveTurningPathController::GetPath(Spline::PiecewisePolynomialND& _path) const
{
    Spline::PiecewisePolynomialND front;
    path.Split(pathOffset,front,_path);
}

void ValveTurningPathController::Cut(Real time,bool relative)
{
    if(relative)
        path.TrimBack(pathOffset+time);
    else
        path.TrimBack(time);
}

Config ValveTurningPathController::Endpoint() const
{
    return path.End();
}

Vector ValveTurningPathController::EndpointVelocity() const
{
    return path.Derivative(path.EndTime());
}

bool ValveTurningPathController::Done() const
{
    return pathOffset >= path.EndTime();
}

Real ValveTurningPathController::TimeRemaining() const
{
    return path.EndTime() - pathOffset;
}

void ValveTurningPathController::GetDesiredState(Config& q_des,Vector& dq_des)
{
    q_des = path.Evaluate(pathOffset);
    dq_des = path.Derivative(pathOffset);
}

void ValveTurningPathController::Update(Real dt)
{
//    cout << robot.name << __func__ << " in (ValveTurningPathController)" << " from " << typeid(*this).name() << endl;
//    cout << "pathOffset : " << pathOffset << endl;
    pathOffset += dt;
    //keep the path relatively short
    if((pathOffset - path.StartTime()) > Max(0.1,0.1*(path.EndTime()-path.StartTime())))
        path.TrimFront(pathOffset);

    ValveTurningJointController::Update(dt);
}

void ValveTurningPathController::Reset()
{
    path = Spline::Constant(path.Evaluate(pathOffset),0,0);
    pathOffset = 0;
}

bool ValveTurningPathController::ReadState(File& f)
{
    if(!ReadFile(f,pathOffset)) return false;
    if(!path.Read(f)) return false;
    return true;
}

bool ValveTurningPathController::WriteState(File& f) const
{
    if(!WriteFile(f,pathOffset)) return false;
    if(!path.Write(f)) return false;
    return true;
}

vector<string> ValveTurningPathController::Commands() const
{
    vector<string> res;
    res.push_back("set_tq");
    res.push_back("set_q");
    res.push_back("set_qv");
    res.push_back("set_v");
    res.push_back("append_tq");
    res.push_back("append_q");
    res.push_back("append_qv");
    res.push_back("brake");
    return res;
}

bool ValveTurningPathController::SendCommand(const string& name,const string& str)
{
    stringstream ss(str);
    Real t;
    Config q,v;
    if(name == "set_tq") {
        ss>>t>>q;
        if(!ss) return false;
        if(t < pathOffset) {
            fprintf(stderr,"set_tq: warning, cut time %g is less than path's endtime %g\n",t,pathOffset);
            return false;
        }
        Cut(0);
        printf("set_tq: Back trimmed to time %g, path parameter %g\n",path.EndTime(),pathOffset);
        Assert(t >= path.EndTime());
        AppendLinear(q,t-path.EndTime());
        return true;
    }
    else if(name == "append_tq") {
        ss>>t>>q;
        if(!ss) return false;
        if(t < path.EndTime()) {
            fprintf(stderr,"append_tq: warning, append time %g is less than path's endtime %g\n",t,path.EndTime());
            return false;
        }
        AppendLinear(q,t-path.EndTime());
        return true;
    }
    else if(name == "set_q") {
        ss>>q;
        if(!ss) return false;
        Cut(0);
        AppendRamp(q);
        return true;
    }
    else if(name == "append_q") {
        ss>>q;
        if(!ss) return false;
        AppendRamp(q);
        return true;
    }
    else if(name == "set_qv") {
        ss>>q>>v;
        if(!ss) return false;
        Cut(0);
        AppendRamp(q,v);
        return true;
    }
    else if(name == "append_qv") {
        ss>>q>>v;
        if(!ss) return false;
        AppendRamp(q,v);
        return true;
    }
    else if(name == "brake") {
        //Brake();
        fprintf(stderr,"Brake is not done yet\n");
        return false;
        return true;
    }
    return false;
}
