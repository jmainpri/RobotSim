#include "RobotPoseWidget.h"
#include <GLdraw/drawextra.h>
#include <robotics/IKFunctions.h>

RobotLinkPoseWidget::RobotLinkPoseWidget()
  :robot(NULL),viewRobot(NULL),highlightColor(1,1,0,1),hoverLink(-1)
{}

RobotLinkPoseWidget::RobotLinkPoseWidget(Robot* _robot,ViewRobot* _viewRobot)
  :robot(_robot),viewRobot(_viewRobot),poseConfig(_robot->q),highlightColor(1,1,0,1),hoverLink(-1)
{}


void RobotLinkPoseWidget::Set(Robot* _robot,ViewRobot* _viewRobot)
{
  robot = _robot;
  viewRobot = _viewRobot;
  poseConfig = _robot->q;
}

bool RobotLinkPoseWidget::Hover(int x,int y,Camera::Viewport& viewport,double& distance) 
{ 
  Ray3D r;
  viewport.getClickSource(x,y,r.source);
  viewport.getClickVector(x,y,r.direction);
  int oldHoverLink = hoverLink;
  distance = Inf;    
  hoverLink = -1;
  Vector3 worldpt;
  robot->UpdateConfig(poseConfig);
  robot->UpdateGeometry();
  for(size_t i=0;i<robot->links.size();i++) {
    if(robot->geometry[i].tris.empty()) continue;
    if(RayCast(robot->geometry[i],r,worldpt) >= 0) {
      Real dist = dot(worldpt-r.source,r.direction);
      if(dist < distance) {
	distance = dist;
	robot->links[i].T_World.mulInverse(worldpt,hoverPt);
	hoverLink = i;
      }
    }
  }
  if(hoverLink != oldHoverLink) Refresh();
  return (hoverLink != -1);
}

bool RobotLinkPoseWidget::BeginDrag(int x,int y,Camera::Viewport& viewport,double& distance) 
{ 
  if(!Hover(x,y,viewport,distance)) return false;
  return true;
}

void RobotLinkPoseWidget::Drag(int dx,int dy,Camera::Viewport& viewport)
{
  robot->UpdateConfig(poseConfig);
  for(size_t i=0;i<robot->drivers.size();i++)
    if(robot->DoesDriverAffect(i,hoverLink)) {
      Real val = Clamp(robot->GetDriverValue(i)+dy*0.02,robot->drivers[i].qmin,robot->drivers[i].qmax);
      robot->SetDriverValue(i,val);
    }
  poseConfig = robot->q;
}

void RobotLinkPoseWidget::DrawGL(Camera::Viewport& viewport) 
{
  if(hasHighlight || hasFocus) {
    if(hoverLink >= 0)
      viewRobot->colors[hoverLink] = highlightColor;
  }
}


RobotIKPoseWidget::RobotIKPoseWidget(Robot* _robot)
  :robot(_robot) 
{}

void RobotIKPoseWidget::ClearLink(int link) 
{
  for(size_t i=0;i<poseGoals.size();i++) {
    if(poseGoals[i].link == link) {
      poseGoals.erase(poseGoals.begin()+i);
      poseWidgets.erase(poseWidgets.begin()+i);
      i--;
    }
  }
  RefreshWidgets();
}
void RobotIKPoseWidget::FixPoint(int link,const Vector3& localpt) 
{
  poseGoals.resize(poseGoals.size()+1);
  poseGoals.back().link = link;
  poseGoals.back().destLink = -1;
  poseGoals.back().localPosition = localpt;
  poseGoals.back().SetFixedPosition(robot->links[link].T_World*localpt);
  poseWidgets.resize(poseGoals.size());
  poseWidgets.back().T.R = robot->links[link].T_World.R;
  poseWidgets.back().T.t = robot->links[link].T_World*localpt;
  poseWidgets.back().enableRotation = false;
  RefreshWidgets();
}
void RobotIKPoseWidget::FixLink(int link) 
{
  poseGoals.resize(poseGoals.size()+1);
  poseGoals.back().link = link;
  poseGoals.back().destLink = -1;
  poseGoals.back().localPosition = robot->links[link].com;
  poseGoals.back().SetFixedPosition(robot->links[link].T_World*robot->links[link].com);
  poseGoals.back().SetFixedRotation(robot->links[link].T_World.R);
  poseWidgets.resize(poseGoals.size());
  poseWidgets.back().T.R = robot->links[link].T_World.R;
  poseWidgets.back().T.t = robot->links[link].T_World*robot->links[link].com;
  poseWidgets.back().enableRotation = true;
  RefreshWidgets();
}
void RobotIKPoseWidget::Add(const IKGoal& goal) {
  poseGoals.push_back(goal);
  poseWidgets.resize(poseWidgets.size()+1);
  poseWidgets.back().T.R = robot->links[goal.link].T_World.R;
  poseWidgets.back().T.t = poseGoals.back().endPosition;  
  if(goal.rotConstraint == IKGoal::RotNone)
    poseWidgets.back().enableRotation = false;
  else
    poseWidgets.back().enableRotation = true;
  RefreshWidgets();
}
void RobotIKPoseWidget::AttachWidget(int widget,int link) 
{
  printf("Attaching widget from link %d to %d\n",poseGoals[widget].destLink,link);
  Assert(widget >= 0 && widget < (int)poseGoals.size());
  int oldDest = poseGoals[widget].destLink;
  poseGoals[widget].destLink = link;
  Matrix3 oldR;
  Vector3 oldp;
  if(oldDest >= 0)
    oldp = robot->links[oldDest].T_World*poseGoals[widget].endPosition;
  else
    oldp = poseGoals[widget].endPosition;
  if(poseGoals[widget].rotConstraint == IKGoal::RotFixed) {
    RigidTransform Tgoal;
    poseGoals[widget].GetFixedGoalTransform(Tgoal);
    if(oldDest >= 0) 
      oldR = robot->links[oldDest].T_World.R*Tgoal.R;
    else
      oldR = Tgoal.R;
  }
  if(link >= 0) {
    robot->links[link].T_World.mulInverse(oldp,poseGoals[widget].endPosition);
    if(poseGoals[widget].rotConstraint == IKGoal::RotFixed) {
      //cout<<"Desired world rotation "<<oldR<<endl;
      //cout<<"Reference link rotation "<<robot->links[link].T_World.R<<endl;
      Matrix3 Rlocal;
      Rlocal.mulTransposeA(robot->links[link].T_World.R,oldR);
      //cout<<"Desired local rotation "<<Rlocal<<endl;
      poseGoals[widget].SetFixedRotation(Rlocal);
    }
  }
  else {
    poseGoals[widget].endPosition = oldp;
    if(poseGoals[widget].rotConstraint == IKGoal::RotFixed) 
      poseGoals[widget].SetFixedRotation(oldR);
  }
}
void RobotIKPoseWidget::RefreshWidgets() 
{
  widgets.resize(poseWidgets.size());
  for(size_t i=0;i<widgets.size();i++) {
    widgets[i] = &poseWidgets[i];
  }
  closestWidget = NULL;
  activeWidget = NULL;
  Refresh();
}
int RobotIKPoseWidget::HoverWidget() const
{
  int index = -1;
  for(size_t i=0;i<poseGoals.size();i++)
    if(closestWidget == &poseWidgets[i]) {
      index = (int)i;
      break;
    }
  return index;
}
int RobotIKPoseWidget::ActiveWidget() const
{
  int index = -1;
  for(size_t i=0;i<poseGoals.size();i++)
    if(activeWidget == &poseWidgets[i]) {
      index = (int)i;
      break;
    }
  return index;
}
bool RobotIKPoseWidget::ClearCurrent()
{
  int index = -1;
  for(size_t i=0;i<poseGoals.size();i++)
    if(activeWidget == &poseWidgets[i]) {
      index = (int)i;
      break;
    }
  if(index != -1) {
    printf("Deleting IK goal on link %s\n",robot->LinkName(poseGoals[index].link).c_str());
    poseGoals.erase(poseGoals.begin()+index);
    poseWidgets.erase(poseWidgets.begin()+index);
    RefreshWidgets();
    return true;
  }
  return false;
}

bool RobotIKPoseWidget::Hover(int x,int y,Camera::Viewport& viewport,double& closestDistance)
{
  if(WidgetSet::Hover(x,y,viewport,closestDistance)) {
    closestDistance -= 1.0;
    return true;
  }
  return false;
}

bool RobotIKPoseWidget::BeginDrag(int x,int y,Camera::Viewport& viewport,double& closestDistance)
{
  if(WidgetSet::BeginDrag(x,y,viewport,closestDistance)) {
    closestDistance -= 1.0;
    return true;
  }
  return false;
}

void RobotIKPoseWidget::DrawGL(Camera::Viewport& viewport) 
{
  WidgetSet::DrawGL(viewport);
  glPolygonOffset(0,-1000);
  glEnable(GL_POLYGON_OFFSET_FILL);
  for(size_t i=0;i<poseGoals.size();i++) {
    Vector3 curpos = robot->links[poseGoals[i].link].T_World*poseGoals[i].localPosition;
    Vector3 despos = poseGoals[i].endPosition;
    if(poseGoals[i].destLink >= 0)
      despos = robot->links[poseGoals[i].destLink].T_World*despos;
    glDisable(GL_LIGHTING);
    glColor3f(1,0,0);
    glLineWidth(5.0);
    glBegin(GL_LINES);
    glVertex3v(curpos);
    glVertex3v(despos);
    glEnd();
    glLineWidth(1.0);
      
    poseWidgets[i].DrawGL(viewport);
      
    float color2[4] = {1,0.5,0,1};
    glMaterialfv(GL_FRONT,GL_AMBIENT_AND_DIFFUSE,color2); 
    glPushMatrix();
    if(poseGoals[i].rotConstraint == IKGoal::RotFixed) {
      RigidTransform T;
      poseGoals[i].GetFixedGoalTransform(T);
      if(poseGoals[i].destLink >= 0)
	T = robot->links[poseGoals[i].destLink].T_World*T;
      glMultMatrix(Matrix4(T));
      drawBox(0.04,0.04,0.04);
    }
    else {
      glTranslate(despos);
      drawSphere(0.02,16,8);
    }
    glPopMatrix();
  }
  glDisable(GL_POLYGON_OFFSET_FILL);
}

void RobotIKPoseWidget::Drag(int dx,int dy,Camera::Viewport& viewport)
{
  if(activeWidget) {
    activeWidget->Drag(dx,dy,viewport);
    Refresh();
    int index = ActiveWidget();
    if(index < 0) return;
    if(poseGoals[index].rotConstraint == IKGoal::RotFixed) {
      poseGoals[index].SetFixedRotation(poseWidgets[index].T.R);
      poseGoals[index].SetFixedPosition(poseWidgets[index].T.t);
    }
    else {
      poseGoals[index].SetFixedPosition(poseWidgets[index].T.t);
    }
  }
}

RobotPoseWidget::RobotPoseWidget()
  :useBase(false),linkPoser(),ikPoser(NULL),poseIKMode(false),attachIKMode(false)
{
  useBase = 0;
}

RobotPoseWidget::RobotPoseWidget(Robot* robot,ViewRobot* viewRobot)
  :useBase(false),linkPoser(robot,viewRobot),ikPoser(robot),poseIKMode(false),attachIKMode(false)
{
  if(robot->joints[0].type == RobotJoint::Floating) {
    useBase=true;
    basePoser.T = robot->links[robot->joints[0].linkIndex].T_World;
  }
  if(useBase) {
    widgets.resize(3);
    widgets[0]=&basePoser;
    widgets[1]=&linkPoser;
    widgets[2]=&ikPoser;
  }
  else {
    widgets.resize(2);
    widgets[0]=&linkPoser;
    widgets[1]=&ikPoser;
  }
}

void RobotPoseWidget::Set(Robot* robot,ViewRobot* viewRobot)
{
  linkPoser.Set(robot,viewRobot);
  ikPoser.robot = robot;
  if(robot->joints[0].type == RobotJoint::Floating) {
    useBase=true;
    basePoser.T = robot->links[robot->joints[0].linkIndex].T_World;
  }
  if(useBase) {
    widgets.resize(3);
    widgets[0]=&basePoser;
    widgets[1]=&linkPoser;
    widgets[2]=&ikPoser;
  }
  else {
    widgets.resize(2);
    widgets[0]=&linkPoser;
    widgets[1]=&ikPoser;
  }
}

  
bool RobotPoseWidget::FixCurrent() 
{
  if(linkPoser.hoverLink >= 0) {
    ikPoser.ClearLink(linkPoser.hoverLink);
    ikPoser.FixLink(linkPoser.hoverLink);
    return true;
  }
  return false;
}

bool RobotPoseWidget::FixCurrentPoint() 
{
  if(linkPoser.hoverLink >= 0) {
    ikPoser.ClearLink(linkPoser.hoverLink);
    ikPoser.FixPoint(linkPoser.hoverLink,linkPoser.hoverPt);
    return true;
  }
  return false;
}

bool RobotPoseWidget::DeleteConstraint()
{
  if(activeWidget == &ikPoser) {
    return ikPoser.ClearCurrent();
  }
  else if(linkPoser.hoverLink >= 0) {
    ikPoser.ClearLink(linkPoser.hoverLink);
    return true;
  }
  return false;
}

bool RobotPoseWidget::ToggleAttach()
{
  attachIKMode = !attachIKMode;
  return attachIKMode;
}

bool RobotPoseWidget::TogglePoseIK()
{
  poseIKMode = !poseIKMode;
  return poseIKMode;
}

void RobotPoseWidget::SetPose(const Config& q)
{
  Robot* robot=linkPoser.robot;
  linkPoser.poseConfig = q;
  if(q != robot->q)
    robot->UpdateConfig(q);
  if(useBase)
    basePoser.T = robot->links[robot->joints[0].linkIndex].T_World;
}

void RobotPoseWidget::DrawGL(Camera::Viewport& viewport)
{
  WidgetSet::DrawGL(viewport);
  if(attachIKMode && hasFocus) {
    //draw a line
    if(ikPoser.ActiveWidget() >= 0) {
      Vector3 x;
      attachRay.closestPoint(ikPoser.poseWidgets[ikPoser.ActiveWidget()].T.t,x);
      glColor3f(1,0.5,0);
      glDisable(GL_LIGHTING);
      glLineWidth(3.0);
      glBegin(GL_LINES);
      glVertex3v(ikPoser.poseWidgets[ikPoser.ActiveWidget()].T.t);
      glVertex3v(x);
      glEnd();
    }
  }
}

bool RobotPoseWidget::BeginDrag(int x,int y,Camera::Viewport& viewport,double& distance)
{
  if(attachIKMode) {
    bool res = ikPoser.Hover(x,y,viewport,distance);
    if(!res) {
      attachIKMode = false;
      return false;    
    }
    attachx=x;
    attachy=y;
    Refresh();
    return true;
  }
  else if(poseIKMode) {
    bool res=WidgetSet::BeginDrag(x,y,viewport,distance);
    if(!res) return false;
    if(closestWidget == &linkPoser) {
      printf("Adding new fixed point constraint\n");
      res=FixCurrentPoint();
      ikPoser.poseWidgets.back().Hover(x,y,viewport,distance);
      ikPoser.poseWidgets.back().SetHighlight(true);
      //following lines let it be dragged immediately -- comment out if you want it fixed
      if(ikPoser.poseWidgets.back().BeginDrag(x,y,viewport,distance)) {
	closestWidget = &ikPoser;
	ikPoser.closestWidget = &ikPoser.poseWidgets.back();
	return true;
      }
    }
    return true;
  }
  else {
    return WidgetSet::BeginDrag(x,y,viewport,distance);
  }
}

void RobotPoseWidget::Drag(int dx,int dy,Camera::Viewport& viewport)
{
  if(attachIKMode) {
    //printf("Attach dragging, hover widget %d\n",ikPoser.ActiveWidget());
    attachx += dx;
    attachy += dy; 
    viewport.getClickSource(attachx,attachy,attachRay.source);
    viewport.getClickVector(attachx,attachy,attachRay.direction);
    double dist;
    bool res=linkPoser.Hover(attachx,attachy,viewport,dist);
    Refresh();
    return;
  }
  WidgetSet::Drag(dx,dy,viewport);
  if(activeWidget == &basePoser) {
    linkPoser.robot->SetJointByTransform(0,5,basePoser.T);
    linkPoser.robot->UpdateFrames();
    linkPoser.poseConfig = linkPoser.robot->q;
  }
  if(activeWidget == &ikPoser) {
    //solve the IK problem
    Robot* robot=linkPoser.robot;
    robot->UpdateConfig(linkPoser.poseConfig);
    int iters=100;
    bool res=SolveIK(*robot,ikPoser.poseGoals,1e-3,iters,0);
    linkPoser.poseConfig = robot->q;
    if(useBase)
      basePoser.T = robot->links[robot->joints[0].linkIndex].T_World;
    Refresh();
  }
}

void RobotPoseWidget::EndDrag()
{
  if(attachIKMode) {
    attachIKMode = false;
    cout<<"Attaching constraint to "<<linkPoser.hoverLink<<endl;
    Refresh();
    int link = linkPoser.hoverLink;
    int widget = ikPoser.ActiveWidget();
    ikPoser.robot->UpdateConfig(linkPoser.poseConfig);
    ikPoser.AttachWidget(widget,link);
  }
  WidgetSet::EndDrag();
}

void RobotPoseWidget::Keypress(char c)
{
  if(c=='s') {
    //solve the IK problem    
    Robot* robot=linkPoser.robot;
    robot->UpdateConfig(linkPoser.poseConfig);
    int iters=100;
    bool res=SolveIK(*robot,ikPoser.poseGoals,1e-3,iters,0);
    linkPoser.poseConfig = robot->q;
    if(useBase)
      basePoser.T = robot->links[robot->joints[0].linkIndex].T_World;
    Refresh();
  }
}
