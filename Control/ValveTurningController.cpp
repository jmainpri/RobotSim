#include "ValveTurningController.h"
#include "Modeling/DynamicPath.h"
#include "Modeling/Conversions.h"
#include "GLdraw/drawextra.h"
#include "Modeling/World.h"

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
    qdesDefault = robot.q;

#ifdef OPCONTROL
    op_space_controller_ = new OpSpaceControl::DRCHuboOpSpace();
    op_space_controller_->SetRobot( &robot );
    op_space_controller_->SetLinkNames( robot.linkNames );
    op_space_controller_->SetRobotNbDofs( robot.q.n );
#endif
    start_ = true;

    // Uncomment to print joint mapping
    for(int i=0; i<int(robot.links.size());i++){
        cout << i << " : " << robot.LinkName(i) << endl;
    }

    q_des_ = Vector(0);
}

//subclasses should override this
void ValveTurningJointController::GetDesiredState( Config& q_des,Vector& dq_des )
{
    q_des = qdesDefault;
    dq_des.setZero();
}

void ValveTurningJointController::Draw()
{
#ifdef OPCONTROL
    op_space_controller_->Draw();
#endif
    if( !q_des_.empty() )
    {
        //draw desired milestone
        robot.UpdateConfig( q_des_ );
        glEnable(GL_LIGHTING);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
        for(size_t j=0;j<robot.links.size();j++)
        {
            glPushMatrix();
            GLDraw::glMultMatrix( Matrix4(robot.links[j].T_World) );
            float color[4] = {1,1,0,0.5};
            glMaterialfv( GL_FRONT, GL_AMBIENT_AND_DIFFUSE, color );
            world_->robots[0].view.DrawLink_Local(j);
            glPopMatrix();
        }
        glDisable(GL_BLEND);
    }

//    // draws the time
//    int w=glutGet(GLUT_WINDOW_WIDTH);
//    int h=glutGet(GLUT_WINDOW_HEIGHT);
//    glMatrixMode(GL_PROJECTION);
//    //glPushMatrix()
//    glLoadIdentity();
//    gluOrtho2D(0, w, 0, h);
//    glScalef(1, -1, 1);
//    glTranslatef(0, -h, 0);
//    glMatrixMode(GL_MODELVIEW);
//    glPushMatrix();
//    glLoadIdentity();
//    glRasterPos2f( 10, 20 );
//    text = "%0.3f" % self.sim.getTime()+ " sec.";
//    for(int i=0;i<text.size();i++)
//    {
//        glColor3f(1,1,1);
//        glutBitmapCharacter( GLUT_BITMAP_9_BY_15, ctypes.c_int( ord(ch) ) );
//    }
//    glPopMatrix();
//    glMatrixMode(GL_PROJECTION);
//    //glPopMatrix()
//    glMatrixMode(GL_MODELVIEW);
}

void ValveTurningJointController::Update(Real dt)
{
    //Assert(false);
    Assert(command != NULL);

    Config qdes(robot.links.size());
    Config dqdes(robot.links.size());
    GetDesiredState( qdes, dqdes );
    robot.NormalizeAngles(qdes);

    // Sotre Q des for drawing
    q_des_ = qdes;

//    qdes = Config(robot.links.size(),0.0);
//    dqdes = Config(robot.links.size(),0.0);

//    qdes[GetLinkIdByName(robot,"Body_LEP")] = -0.3;
//    qdes[GetLinkIdByName(robot,"Body_REP")] = -0.3;

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

        //cout << q_cur << endl;

        if( start_ )
        {
#ifdef OPCONTROL
            op_space_controller_->CreateTasks( q_cur, dt );
#endif
            start_ = false;
            //cout << GetLinkByName(robot,"Body_TSY").T_World << endl;
        }

        if( time > 0.05 )
        {
            /*
            cout << "call to op space control" << endl;
            std::string configuration("61 7.931164611074985e-05 5.085316123923096e-06 -0.004575521194602139 2.8765031127972157e-06 0.00030361989008492475 6.2831386914885465 0.0006302653622816123 -0.00017248098151867453 -3.716289373745241e-05 -0.0018322754627346782 0.0003517744447689708 0.0015318113971227731 -3.962696837334079e-11 4.049205415412871e-12 9.00612917575927e-13 2.913225216616411e-13 -4.780176254826074e-12 -1.2603251775544777e-12 -3.934630399271555e-13 3.611333454500709e-12 9.103828801926284e-13 2.504663143554353e-13 0.0 -5.7990945379060577e-11 -0.0004807511721010016 -0.0011029157226714048 0.0006085579888246428 1.2615671124649452e-05 -0.00027755650699390344 -0.0017582150092927407 1.5982656528912287e-05 0.001747605915071837 2.6498803151753236e-11 5.917044632042234e-12 8.206768598029157e-13 2.3714363805993344e-13 -5.474731779031572e-12 -2.1316282072803006e-13 -8.08242361927114e-14 -3.1441516057384433e-12 -9.530154443382344e-13 -3.0819791163594346e-13 -4.905409412003792e-12 -1.6289192217300297e-12 -4.867217739956686e-13 0.0 -1.699484013961694e-06 -5.1821102076132775e-08 3.40193315588877e-05 -0.00012555444109985103 -0.0003754183378124054 -2.7150197580816382e-05 0.0002540384996390088 0.0 -1.789444290523079e-07 3.0286636642173903e-05 -9.170349027964875e-05 -0.00039266028179874013 -0.00015156556474504868 4.2285917150053365e-06 0.0");
            std::stringstream in( configuration.c_str());
            in >> q_cur;
            */

            // Run OpSpace controller
            std::pair< std::vector<double> , std::vector<double> > q_opspace_;
#ifdef OPCONTROL
            q_opspace_ = op_space_controller_->Trigger( q_cur, dq_cur, qdes, dqdes, dt );
#endif
            qdes    = q_opspace_.first; // modifed configuration
            dqdes   = q_opspace_.second; // modified velocity

//            cout << "q_cur" << endl;
//            cout << q_cur << endl;

//            cout << "dt : " << dt << endl;
//            cout << "qdes : " << qdes << endl;
//            cout << "dqdes : " << dqdes << endl;

            //exit(0);

            for( size_t i=0; i<robot.drivers.size(); i++ )
            {
                if( robot.drivers[i].type == RobotJointDriver::Normal )
                {
                    command->actuators[i].SetPID(qdes(robot.drivers[i].linkIndices[0]), dqdes(robot.drivers[i].linkIndices[0]), command->actuators[i].iterm);
                }
                else
                {
                    robot.q = qdes;
                    robot.dq = dqdes;
                    cout << "Desired affine driver value " << robot.GetDriverValue(i) <<", " << robot.GetDriverVelocity(i) << endl;
                    command->actuators[i].SetPID( robot.GetDriverValue(i), robot.GetDriverVelocity(i), command->actuators[i].iterm );
                }
            }
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
