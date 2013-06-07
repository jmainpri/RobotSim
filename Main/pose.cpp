#include "ResourceBrowserProgram.h"
#include "Planning/RobotCSpace.h"
#include "Planning/RobotConstrainedInterpolator.h"
#include "Planning/RobotTimeScaling.h"
#include "Modeling/MultiPath.h"
#include "Modeling/Interpolate.h"
#include "Contact/Utils.h"
#include <robotics/IKFunctions.h>
#include <meshing/Voxelize.h>
#include <meshing/MarchingCubes.h>
#include <GLdraw/GLLight.h>
#include "View/RobotPoseWidget.h"
//#include <GL/glut.h>
#include <math/random.h>
#include <fstream>
using namespace Math3D;
using namespace GLDraw;

//HD 720p settings
const static int MOVIE_W = 1280;
const static int MOVIE_H = 720;

//VGA
//const static int MOVIE_W = 640;
//const static int MOVIE_H = 480;

enum {
  STORE_TO_LIBRARY_ID = RESOURCE_BROWSER_GLUI_ID_END,
  OVERWRITE_LIBRARY_ID,
  RETRIEVE_FROM_LIBRARY_ID,
  CREATE_PATH_ID,
  SAVE_PATH_CONFIGS_ID,
  OPTIMIZE_PATH_ID,
  SAVE_MOVIE_BUTTON_ID,
  POSE_IK_TOGGLED_ID,
  LINK_SPINNER_ID,
  LINK_LISTBOX_ID,
  LINK_VALUE_SPINNER_ID,
  DRIVER_SPINNER_ID,
  DRIVER_LISTBOX_ID,
  DRIVER_VALUE_SPINNER_ID,
  IK_UPDATE_ID,
};

enum { CONFIG_TYPE, CONFIGS_TYPE, IK_TYPE, STANCE_TYPE, GRASP_TYPE, MULTIPATH_TYPE, LINEAR_PATH_TYPE };

class RobotPoseProgram : public ResourceBrowserProgram
{
public:
  //internal state
  int cur_link,cur_driver;
  vector<bool> self_colliding;

  //GUI state
  GLUI* glui;
  GLUI_Spinner* link_spinner, *link_value_spinner;
  GLUI_Listbox* link_listbox;
  GLUI_Spinner* driver_spinner, *driver_value_spinner;
  GLUI_Listbox* driver_listbox;
  float link_value;
  float driver_value;
  RobotPoseWidget poseWidget;
  int pose_ik;
  int draw_geom,draw_com,draw_frame;

  RobotPoseProgram(RobotWorld* world)
    :ResourceBrowserProgram(world)
  {}

  virtual bool Initialize()
  {
    poseWidget.Set(world->robots[0].robot,&world->robots[0].view);
    cur_link=0;
    cur_driver=0;
    link_value = 0;
    driver_value = 0;
    draw_geom = 1;
    draw_com = 0;
    draw_frame = 0;
    pose_ik = 0;
    if(!ResourceBrowserProgram::Initialize()) return false;
    Robot* robot = world->robots[0].robot;
    viewResource.SetRobot(robot);
    self_colliding.resize(robot->links.size(),false);   
    UpdateConfig();

    //setup GUI 
    glui = GLUI_Master.create_glui_subwindow(main_window,GLUI_SUBWINDOW_RIGHT);
    glui->set_main_gfx_window(main_window);

    ResourceBrowserProgram::AddGLUIControls(glui);

    glui->add_button("Library -> Poser",RETRIEVE_FROM_LIBRARY_ID,ControlFunc);
    glui->add_button("Poser -> Library",STORE_TO_LIBRARY_ID,ControlFunc);
    glui->add_button("Poser -> Overwrite",OVERWRITE_LIBRARY_ID,ControlFunc);
    glui->add_checkbox("Pose by IK",&pose_ik,POSE_IK_TOGGLED_ID,ControlFunc);
    GLUI_Panel* panel = glui->add_rollout("Path controls");
    glui->add_button_to_panel(panel,"Create path",CREATE_PATH_ID,ControlFunc);
    glui->add_button_to_panel(panel,"Save configs",SAVE_PATH_CONFIGS_ID,ControlFunc);
    glui->add_column_to_panel(panel,false);
    glui->add_button_to_panel(panel,"Optimize path",OPTIMIZE_PATH_ID,ControlFunc);
    glui->add_button_to_panel(panel,"Save movie",SAVE_MOVIE_BUTTON_ID,ControlFunc);

    panel = glui->add_rollout("Link controls");
    link_spinner = glui->add_spinner_to_panel(panel,"Index",GLUI_SPINNER_INT,&cur_link,LINK_SPINNER_ID,ControlFunc);
    link_spinner->set_int_limits(0,robot->links.size()-1,GLUI_LIMIT_WRAP);

    link_listbox = glui->add_listbox_to_panel(panel,"Name",&cur_link,LINK_LISTBOX_ID,ControlFunc);
    for(size_t i=0;i<robot->links.size();i++) {
      char buf[256];
      strcpy(buf,robot->linkNames[i].c_str());
      link_listbox->add_item(i,buf);
    }

    link_value_spinner = glui->add_spinner_to_panel(panel,"Angle",GLUI_SPINNER_FLOAT,&link_value,LINK_VALUE_SPINNER_ID,ControlFunc);
    UpdateLinkValueGUI();

    panel = glui->add_rollout("Driver controls");
    driver_spinner = glui->add_spinner_to_panel(panel,"Index",GLUI_SPINNER_INT,&cur_driver,DRIVER_SPINNER_ID,ControlFunc);
    driver_spinner->set_int_limits(0,robot->drivers.size()-1,GLUI_LIMIT_WRAP);

    driver_listbox = glui->add_listbox_to_panel(panel,"Name",&cur_driver,DRIVER_LISTBOX_ID,ControlFunc);
    for(size_t i=0;i<robot->drivers.size();i++) {
      char buf[256];
      strcpy(buf,robot->driverNames[i].c_str());
      driver_listbox->add_item(i,buf);
    }

    driver_value_spinner = glui->add_spinner_to_panel(panel,"Angle",GLUI_SPINNER_FLOAT,&driver_value,DRIVER_VALUE_SPINNER_ID,ControlFunc);

    glui->add_checkbox("Draw geometry",&draw_geom);
    glui->add_checkbox("Draw frames",&draw_frame);
    glui->add_checkbox("Draw COM",&draw_com);
    UpdateDriverValueGUI();

    ResourceBrowserProgram::Add("Initial",robot->q);
    GLColor configColor(0.5,0.5,0.5,0.25);
    ResourceBrowserProgram::viewResource.SetRobot(robot);
    ResourceBrowserProgram::viewResource.configViewer.SetColors(configColor);
    ResourceBrowserProgram::SetActive("Config","Initial");
    ResourceBrowserProgram::RefreshCurrent();

    SleepIdleCallback();
    return true;
  }
  
  void UpdateConfig()
  {
    Robot* robot = world->robots[0].robot;
    robot->UpdateConfig(poseWidget.Pose());

    //update collisions
    for(size_t i=0;i<robot->links.size();i++)
      self_colliding[i]=false;
    robot->UpdateGeometry();
    for(size_t i=0;i<robot->links.size();i++) {
      for(size_t j=0;j<robot->links.size();j++) {
	if(robot->SelfCollision(i,j)) {
	  self_colliding[i]=self_colliding[j]=true;
	}
      }
    }
  }

  void UpdateLinkSpinnerGUI()
  {
    link_spinner->set_int_val(cur_link);
    //driver_spinner->set_int_val(cur_driver);
  }

  void UpdateLinkListboxGUI()
  {
    link_listbox->set_int_val(cur_link);
    //driver_listbox->set_int_val(cur_driver);
  }

  void UpdateCurLinkGUI()
  {
    UpdateLinkSpinnerGUI();
    UpdateLinkListboxGUI();
  }

  void UpdateLinkValueGUI()
  {
    Robot* robot = world->robots[0].robot;
    Vector2 limits(robot->qMin(cur_link),robot->qMax(cur_link));
    link_value_spinner->set_float_limits(limits.x,limits.y);
    link_value = robot->q(cur_link);
  }

  void UpdateDriverSpinnerGUI()
  {
    driver_spinner->set_int_val(cur_driver);
  }

  void UpdateDriverListboxGUI()
  {
    driver_listbox->set_int_val(cur_driver);
  }

  void UpdateCurDriverGUI()
  {
    UpdateDriverSpinnerGUI();
    UpdateDriverListboxGUI();
  }

  void UpdateDriverValueGUI()
  {
    Robot* robot = world->robots[0].robot;
    Vector2 limits = robot->GetDriverLimits(cur_driver);
    driver_value_spinner->set_float_limits(limits.x,limits.y);
    driver_value = robot->GetDriverValue(cur_driver);
  }

  virtual void RenderWorld()
  {
    Robot* robot = world->robots[0].robot;
    ViewRobot& viewRobot = world->robots[0].view;
    //ResourceBrowserProgram::RenderWorld();
    for(size_t i=0;i<world->terrains.size();i++)
      world->terrains[i].view.Draw();
    for(size_t i=0;i<world->rigidObjects.size();i++)
      world->rigidObjects[i].view.Draw();

    if(draw_geom) {
      //set the robot colors
      GLColor highlight(1,1,0);
      GLColor driven(1,0.5,0);
      GLColor colliding(1,0,0);
      viewRobot.SetGrey();
      for(size_t i=0;i<robot->links.size();i++) {
	if(self_colliding[i]) viewRobot.colors[i] = colliding;
	else if((int)i == cur_link)
	  viewRobot.colors[i] = highlight; 
      }
      poseWidget.DrawGL(viewport);
      viewRobot.Draw();
    }
    else
      viewRobot.SetGrey();

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
    ResourceBrowserProgram::RenderCurResource();
    glDisable(GL_BLEND);
   
    if(draw_com) {
      viewRobot.DrawCenterOfMass();
      for(size_t i=0;i<robot->links.size();i++)
	viewRobot.DrawLinkCenterOfMass(i,0.01);
    }
    if(draw_frame) {
      viewRobot.DrawLinkFrames();
      glDisable(GL_DEPTH_TEST);
      glPushMatrix();
      glMultMatrix((Matrix4)robot->links[cur_link].T_World);
      drawCoords(0.2);
      glPopMatrix();
      glEnable(GL_DEPTH_TEST);
    }
    /*
    if(!poseGoals.empty()) {
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
    */
  }

  ResourcePtr PoserToResource()
  {
    Robot* robot = world->robots[0].robot;
    string type = resource_types[ResourceBrowserProgram::cur_resource_type];
    if(type == "Config") 
      return MakeResource("",robot->q);
    else if(type == "IKGoal") {
      int ind = poseWidget.ikPoser.ActiveWidget();
      if(ind < 0) {
	printf("Not hovering over any IK widget\n");
	return NULL;
      }
      return MakeResource("",poseWidget.ikPoser.poseGoals[ind]);
    }
    else if(type == "Stance") {
      Stance s;
      if(poseWidget.ikPoser.poseGoals.empty()) {
	printf("Storing flat ground stance\n");
	ContactFormation cf;
	GetFlatContacts(*robot,1e-3,cf);
	for(size_t i=0;i<cf.links.size();i++) {
	  Hold h;
	  LocalContactsToHold(cf.contacts[i],cf.links[i],*robot,h);
	  s.insert(h);
	}
      }
      else {
	printf("Storing flat contact stance\n");
	for(size_t i=0;i<poseWidget.ikPoser.poseGoals.size();i++) {
	  int link = poseWidget.ikPoser.poseGoals[i].link;
	  vector<ContactPoint> cps;
	  GetFlatContacts(*robot,link,1e-3,cps);
	  Hold h;
	  LocalContactsToHold(cps,link,*robot,h);
	  h.ikConstraint = poseWidget.ikPoser.poseGoals[i];
	  s.insert(h);
	}
      }
      return MakeResource("",s);
    }
    else if(type == "Grasp") {
      int link = 0;
      cout<<"Which robot link to use? > "; cout.flush();
      cin >> link;
      cin.ignore(256,'\n');
      Grasp g;
      g.constraints.resize(1);
      g.constraints[0].SetFixedTransform(robot->links[link].T_World);
      g.constraints[0].link = link;
      vector<bool> descendant;
      robot->GetDescendants(link,descendant);
      for(size_t i=0;i<descendant.size();i++)
	if(descendant[i]) {
	  g.fixedDofs.push_back(i);
	  g.fixedValues.push_back(robot->q[i]);
	}
      
      ResourcePtr r=ResourceBrowserProgram::CurrentResource();
      const GeometricPrimitive3DResource* gr = dynamic_cast<const GeometricPrimitive3DResource*>((const ResourceBase*)r);
      if(gr) {
	cout<<"Making grasp relative to "<<gr->name<<endl;
	//TODO: detect contacts
	
	RigidTransform T = gr->data.GetFrame();
	RigidTransform Tinv;
	Tinv.setInverse(T);
	g.Transform(Tinv);
      }
      else {
	const RigidObjectResource* obj = dynamic_cast<const RigidObjectResource*>((const ResourceBase*)r);
	if(obj) {
	  cout<<"Making grasp relative to "<<obj->name<<endl;
	  //TODO: detect contacts
	  
	  RigidTransform T = obj->object.T;
	  RigidTransform Tinv;
	  Tinv.setInverse(T);
	  g.Transform(Tinv);
	}
      }
      return MakeResource("",g);
    }
    else {
      fprintf(stderr,"Poser does not contain items of the selected type\n");
      return NULL;
    }
  }

  virtual void Handle_Control(int id)
  {
    Robot* robot = world->robots[0].robot;
    switch(id) {
    case SAVE_MOVIE_BUTTON_ID:
      //resize for movie
      {
	int totalw = glutGet(GLUT_WINDOW_WIDTH);
	int totalh = glutGet(GLUT_WINDOW_HEIGHT);
	int toolbarw = totalw - viewport.w;
	int toolbarh = totalh - viewport.h;
	glutReshapeWindow(toolbarw+MOVIE_W,toolbarh+MOVIE_H);
      }
      ToggleMovie();
      ResourceBrowserProgram::viewResource.pathTime=0;
      break;
    case STORE_TO_LIBRARY_ID:
      {
	ResourcePtr r=PoserToResource();
	if(r) {
	  ResourceBrowserProgram::Add(r);
	  ResourceBrowserProgram::SetLastActive();
	}
      }
      break;
    case OVERWRITE_LIBRARY_ID:
      {
	ResourcePtr r = PoserToResource();
	ResourcePtr oldr = ResourceBrowserProgram::CurrentResource();
	if(0!=strcmp(oldr->Type(),r->Type())) {
	  printf("Unable to overwrite, selected item is not of the correct type\n");
	  break;
	}
	r->name = oldr->name;
	r->fileName = oldr->fileName;
	vector<ResourcePtr >& v=resources.itemsByType[resource_types[cur_resource_type]];
	v[cur_resource_name] = r;
	vector<ResourcePtr>& v2 = resources.itemsByName[r->name];
	for(size_t i=0;i<v2.size();i++)
	  if(v2[i] == oldr) v2[i] = r;
	last_added = r;
	SetLastActive();
      }
      break;
    case RETRIEVE_FROM_LIBRARY_ID:
      {
	ResourcePtr r=ResourceBrowserProgram::CurrentResource();
	const ConfigResource* rc = dynamic_cast<const ConfigResource*>((const ResourceBase*)r);
	if(rc) {
	  poseWidget.SetPose(rc->data);
	  robot->NormalizeAngles(poseWidget.linkPoser.poseConfig);
	  if(poseWidget.linkPoser.poseConfig != rc->data)
	    printf("Warning: config in library is not normalized\n");
	  UpdateConfig();
	  Refresh();
	}
	else {
	  const IKGoalResource* rc = dynamic_cast<const IKGoalResource*>((const ResourceBase*)r);
	  if(rc) {
	    poseWidget.ikPoser.ClearLink(rc->data.link);
	    poseWidget.ikPoser.Add(rc->data);
	  }
	  else {
	    const StanceResource* rc = dynamic_cast<const StanceResource*>((const ResourceBase*)r);
	    if(rc) {
	      poseWidget.ikPoser.poseGoals.clear();
	      poseWidget.ikPoser.poseWidgets.clear();
	      for(Stance::const_iterator i=rc->stance.begin();i!=rc->stance.end();i++) {
		Assert(i->first == i->second.ikConstraint.link);
		poseWidget.ikPoser.Add(i->second.ikConstraint);
	      }
	      Refresh();
	    }
	    else {
	      const GraspResource* rc = dynamic_cast<const GraspResource*>((const ResourceBase*)r);
	      if(rc) {
		poseWidget.ikPoser.poseGoals.clear();
		poseWidget.ikPoser.poseWidgets.clear();
		Stance s;
		rc->grasp.GetStance(s);
		for(Stance::const_iterator i=s.begin();i!=s.end();i++)
		  poseWidget.ikPoser.Add(i->second.ikConstraint);
		Refresh();
	      }
	    }

	    }
	}
      }
      break;
    case CREATE_PATH_ID:
      {
	vector<Real> times;
	vector<Config> configs,milestones;

	ResourcePtr r=ResourceBrowserProgram::CurrentResource();
	const ConfigResource* rc = dynamic_cast<const ConfigResource*>((const ResourceBase*)r);
	if(rc) {
	  Config a,b;
	  a = rc->data;
	  b = robot->q;
	  if(a.n != b.n) {
	    fprintf(stderr,"Incorrect start and end config size\n");
	    return;
	  }
	  milestones.resize(2);
	  milestones[0] = a;
	  milestones[1] = b;
	  times.resize(2);
	  times[0] = 0;
	  times[1] = 1;
	}
	else {
	  const ConfigsResource* rc = dynamic_cast<const ConfigsResource*>((const ResourceBase*)r);
	  if(rc) {
	    milestones = rc->configs;
	    times.resize(rc->configs.size());
	    for(size_t i=0;i<rc->configs.size();i++)
	      times[i] = Real(i)/(rc->configs.size()-1);
	  }
	  else {
	    return;
	  }
	}
	/*
	if(poseWidget.Constraints().empty()) {
	  //straight line interpolator
	  configs = milestones;
	}
	else {
	  Robot* robot=world->robots[0].robot;
	  Timer timer;
	  if(!InterpolateConstrainedPath(*robot,milestones,poseWidget.Constraints(),configs,1e-2)) return;

	  //int numdivs = (configs.size()-1)*10+1;
	  int numdivs = (configs.size()-1);
	  vector<Real> newtimes;
	  vector<Config> newconfigs;
	  printf("Discretizing at resolution %g\n",1.0/Real(numdivs));
	  SmoothDiscretizePath(*robot,configs,numdivs,newtimes,newconfigs);
	  cout<<"Smoothed to "<<newconfigs.size()<<" milestones"<<endl;
	  cout<<"Total time "<<timer.ElapsedTime()<<endl;
	  swap(times,newtimes);
	  swap(configs,newconfigs);
	}
	ResourceBrowserProgram::Add("",times,configs);
	*/
	MultiPath path;
	path.SetMilestones(milestones);
	path.SetIKProblem(poseWidget.Constraints());
	ResourceBrowserProgram::Add("",path);
	ResourceBrowserProgram::SetLastActive(); 
	ResourceBrowserProgram::viewResource.pathTime = 0;
	Refresh();
      }
      break;
    case SAVE_PATH_CONFIGS_ID:
      {
	ResourcePtr r=ResourceBrowserProgram::CurrentResource();
	const ConfigsResource* cp = dynamic_cast<const ConfigsResource*>((const ResourceBase*)r);
	if(cp) {
	  for(size_t i=0;i<cp->configs.size();i++) {
	    stringstream ss;
	    ss<<cp->name<<"["<<i+1<<"/"<<cp->configs.size()<<"]";
	    Add(ss.str(),cp->configs[i]);
	  }
	  ResourceBrowserProgram::SetLastActive(); 
	  Refresh();
	  break;
	}
	const LinearPathResource* lp = dynamic_cast<const LinearPathResource*>((const ResourceBase*)r);
	if(lp) {
	  int num;
	  cout<<"How many points? > "; cout.flush();
	  cin >> num;
	  cin.ignore(256,'\n');
	  for(int i=0;i<num;i++) {
	    Real t = Real(lp->times.size()-1)*Real(i+1)/(num+1);
	    int seg = (int)Floor(t);
	    Real u = t - Floor(t);
	    Config q;
	    Assert(seg >= 0 && seg+1 <(int)lp->milestones.size());
	    Interpolate(*robot,lp->milestones[seg],lp->milestones[seg+1],u,q);
	    stringstream ss;
	    ss<<lp->name<<"["<<i+1<<"/"<<num<<"]";
	    Add(ss.str(),q);
	  }
	  ResourceBrowserProgram::SetLastActive(); 
	  Refresh();
	  break;
	}
	const MultiPathResource* mp = dynamic_cast<const MultiPathResource*>((const ResourceBase*)r);
	if(mp) {
	  int num;
	  cout<<"How many points? > "; cout.flush();
	  cin >> num;
	  cin.ignore(256,'\n');
	  Real minTime = 0, maxTime = 1;
	  if(mp->path.HasTiming()) {
	    minTime = mp->path.sections.front().times.front();
	    maxTime = mp->path.sections.back().times.back();
	  }
	  Config q;
	  for(int i=0;i<num;i++) {
	    Real t = minTime + (maxTime - minTime)*Real(i+1)/(num+1);
	    EvaluateMultiPath(*robot,mp->path,t,q);
	    stringstream ss;
	    ss<<mp->name<<"["<<i+1<<"/"<<num<<"]";
	    Add(ss.str(),q);
	  }
	  ResourceBrowserProgram::SetLastActive(); 
	  Refresh();
	  break;
	}
      }
      break;
    case OPTIMIZE_PATH_ID:
      {
	ResourcePtr r=ResourceBrowserProgram::CurrentResource();
	const LinearPathResource* lp = dynamic_cast<const LinearPathResource*>((const ResourceBase*)r);
	if(lp) {
	  vector<double> newtimes;
	  vector<Config> newconfigs;
	  if(!TimeOptimizePath(*robot,lp->times,lp->milestones,0.01,newtimes,newconfigs)) {
	    fprintf(stderr,"Error optimizing path\n");
	    return;
	  }
	  ResourceBrowserProgram::Add("",newtimes,newconfigs);
	  ResourceBrowserProgram::SetLastActive(); 
	  ResourceBrowserProgram::viewResource.pathTime = 0;
	}
	const MultiPathResource* mp = dynamic_cast<const MultiPathResource*>((const ResourceBase*)r);
	if(mp) {
	  Real xtol = 0.05;
	  Real dt = 0.01;
	  MultiPath path = mp->path;
	  if(!GenerateAndTimeOptimizeMultiPath(*robot,path,xtol,dt)) {
	    fprintf(stderr,"Error optimizing path\n");
	    return;
	  }
	  ResourceBrowserProgram::Add("",path);
	  ResourceBrowserProgram::SetLastActive(); 
	  ResourceBrowserProgram::viewResource.pathTime = 0;
	}
	const ConfigsResource* rc = dynamic_cast<const ConfigsResource*>((const ResourceBase*)r);
	if(rc) {
	  MultiPath path;
	  path.sections.resize(1);
	  path.sections[0].milestones = rc->configs;
	  path.SetIKProblem(poseWidget.Constraints(),0);
	  Real xtol = 0.05;
	  Real dt = 0.01;
	  if(!GenerateAndTimeOptimizeMultiPath(*robot,path,xtol,dt)) {
	    fprintf(stderr,"Error optimizing path\n");
	    return;
	  }
	  ResourceBrowserProgram::Add("",path);
	  ResourceBrowserProgram::SetLastActive(); 
	  ResourceBrowserProgram::viewResource.pathTime = 0;	  
	}
      }
      break;
    case POSE_IK_TOGGLED_ID:
      poseWidget.poseIKMode = (pose_ik != 0);
      Refresh();
      break;
    case LINK_SPINNER_ID:
      UpdateLinkListboxGUI();
      UpdateLinkValueGUI();
      GLUI_Master.sync_live_all();
      break;
    case LINK_LISTBOX_ID:
      UpdateLinkSpinnerGUI();
      UpdateLinkValueGUI();
      GLUI_Master.sync_live_all();
      break;
    case LINK_VALUE_SPINNER_ID:
      robot->q(cur_link)=link_value;
      UpdateConfig();
      if(robot->DoesDriverAffect(cur_driver,cur_link)) {
	UpdateDriverValueGUI();
	GLUI_Master.sync_live_all();
      }
      break;
    case DRIVER_SPINNER_ID:
      UpdateDriverListboxGUI();
      UpdateDriverValueGUI();
      GLUI_Master.sync_live_all();
      break;
    case DRIVER_LISTBOX_ID:
      UpdateDriverSpinnerGUI();
      UpdateDriverValueGUI();
      GLUI_Master.sync_live_all();
      break;
    case DRIVER_VALUE_SPINNER_ID:
      robot->SetDriverValue(cur_driver,driver_value);
      UpdateConfig();
      if(robot->DoesDriverAffect(cur_driver,cur_link)) {
	UpdateLinkValueGUI();
	GLUI_Master.sync_live_all();
      }
      break;
    default:
      ResourceBrowserProgram::Handle_Control(id);
      break;
    }
  }

  virtual void Handle_Keypress(unsigned char key,int x,int y)
  {
    Robot* robot = world->robots[0].robot;
    switch(key) {
    case 'h':
      printf("Help:\n");
      printf("[space]: next link\n");
      printf("z: previous link\n");
      printf("d: in pose-by-IK mode, delete constraint\n");
      printf("c: in pose-by-IK mode, constrain current rotations\n");
      printf("a: attach hovered IK constraint to another link\n");
      printf("p: print the current configuration\n");
      printf("v: save current viewport\n");
      printf("V: load viewport\n");
      break;
    case ' ':
      cur_link++;
      if(cur_link >= (int)robot->links.size()) cur_link=0;
      UpdateCurLinkGUI();
      UpdateLinkValueGUI();
      break;
    case 'z':
      cur_link--;
      if(cur_link < 0) cur_link = robot->links.size()-1;
      UpdateCurLinkGUI();
      UpdateLinkValueGUI();
      break;
    case 'c':
      if(!poseWidget.FixCurrent())
	  printf("Before constraining a link you need to hover over it\n");
      break;
    case 'a':
      poseWidget.ToggleAttach();
      break;
    case 'd':
      poseWidget.DeleteConstraint();
      break;
    case 'p':
      cout<<robot->q<<endl;
      break;
    case 'v':
      {
	printf("Saving viewport to view.txt\n");
	ofstream out("view.txt",ios::out);
	WriteDisplaySettings(out);
	break;
      }
    case 'V':
      {
	printf("Loading viewport from view.txt...\n");
	ifstream in("view.txt",ios::in);
	if(!in) {
	  printf("Unable to open view.txt\n");
	}
	else {
	  ReadDisplaySettings(in);
	}
	break;
      }
    default:
      poseWidget.Keypress(key);
      break;
    }
    Refresh();
  }

  void Handle_Motion(int x, int y)
  {
    double d;
    if(poseWidget.Hover(x,viewport.h-y,viewport,d))
      poseWidget.SetHighlight(true);
    else
      poseWidget.SetHighlight(false);
    if(poseWidget.requestRedraw) { Refresh(); poseWidget.requestRedraw=false; }
  }

  virtual void BeginDrag(int x,int y,int button,int modifiers)
  {
    Robot* robot = world->robots[0].robot;
    if(button == GLUT_RIGHT_BUTTON) {
      double d;
      if(poseWidget.BeginDrag(x,viewport.h-y,viewport,d))
	poseWidget.SetFocus(true);
      else
	poseWidget.SetFocus(false);
      if(poseWidget.requestRedraw) { Refresh(); poseWidget.requestRedraw=false; }
    }
  }

  virtual void EndDrag(int x,int y,int button,int modifiers)
  {
    if(button == GLUT_RIGHT_BUTTON) {
      if(poseWidget.hasFocus) {
	poseWidget.EndDrag();
	poseWidget.SetFocus(false);
      }
    }
  }

  virtual void DoFreeDrag(int dx,int dy,int button)
  {
    Robot* robot = world->robots[0].robot;
    if(button == GLUT_LEFT_BUTTON)  DragRotate(dx,dy);
    else if(button == GLUT_RIGHT_BUTTON) {
      if(poseWidget.hasFocus) {
	poseWidget.Drag(dx,-dy,viewport);
	if(poseWidget.requestRedraw) {
	  poseWidget.requestRedraw = false;
	  Refresh();
	  UpdateConfig();
	  UpdateLinkValueGUI();
	  UpdateDriverValueGUI();
	}
      }
    }
  }

  virtual void Handle_Idle() {
    //constant animation
    if(!saveMovie) {
      ResourceBrowserProgram::viewResource.pathTime += timer.ElapsedTime();
      timer.Reset();
    }
    else
      ResourceBrowserProgram::viewResource.pathTime = lastScreenshotTime + frameTime;
    Refresh();
    MovieUpdate(ResourceBrowserProgram::viewResource.pathTime+1e-5);
    ResourceBrowserProgram::Handle_Idle();
  }
};

const char* USAGE_STRING = "Usage: RobotPose [options] [robot, terrain, object, world files]\n";
const char* OPTIONS_STRING = "Options:\n\
\t-l [file]: loads the given resource file.\n\
";

#include <utils/EquivalenceMap.h>

struct EqualVertex
{
  Real tol;
  EqualVertex(Real _tol) : tol(_tol) {}
  bool operator () (const Vector3& a,const Vector3& b) const { return a.isEqual(b,tol); }
};

bool EqualTri(const IntTriple& t1,const IntTriple& t2)
{
  if(t1.a == t2.a) {
    return (t1.b == t2.b && t1.c == t2.c);
  }
  else if(t1.a == t2.b) {
    return (t1.b == t2.c && t1.c == t2.a);
  }
  else if(t1.a == t2.c) {
    return (t1.b == t2.a && t1.c == t2.b);
  }
  return false;
}

void CollapseVerts(Meshing::TriMesh& mesh,Real tol=0.0)
{
  EqualVertex eq(tol);
  vector<vector<int> > components;
  EquivalenceMap(mesh.verts,components,eq);
  vector<int> oldToNew(mesh.verts.size());
  for(size_t i=0;i<components.size();i++)
    for(size_t j=0;j<components[i].size();j++)
      oldToNew[components[i][j]] = (int)i;
  vector<Vector3> newverts(components.size());
  for(size_t i=0;i<components.size();i++) {
    Assert(components[i].size() >= 1);
    newverts[i] = mesh.verts[components[i][0]];
    for(size_t j=1;j<components[i].size();j++)
      newverts[i] += mesh.verts[components[i][j]];
    newverts[i] /= components[i].size();
  }
  printf("Deleted %d duplicate vertices\n",mesh.verts.size()-newverts.size());
  mesh.verts = newverts;

  //construct triangles, discarding duplicate triangles
  vector<IntTriple> newtris;
  newtris.reserve(mesh.tris.size());
  vector<vector<int> > incidentTris(newverts.size());
  for(size_t i=0;i<mesh.tris.size();i++) {
    IntTriple tri;
    tri.a = oldToNew[mesh.tris[i].a];
    tri.b = oldToNew[mesh.tris[i].b];
    tri.c = oldToNew[mesh.tris[i].c];
    if(tri.a == tri.b || tri.a == tri.c || tri.c == tri.b) {
      //skip[
    }
    else {
      //look through current tris for duplicates
      bool duplicate = false;
      for(size_t j=0;j<incidentTris[tri.a].size();j++) {
	int t=incidentTris[tri.a][j];
	if(EqualTri(newtris[t],tri)) {
	  duplicate = true;
	  break;
	}
      }
      if(duplicate) continue;
      for(size_t j=0;j<incidentTris[tri.b].size();j++) {
	int t=incidentTris[tri.b][j];
	if(EqualTri(newtris[t],tri)) {
	  duplicate = true;
	  break;
	}
      }
      if(duplicate) continue;
      for(size_t j=0;j<incidentTris[tri.c].size();j++) {
	int t=incidentTris[tri.c][j];
	if(EqualTri(newtris[t],tri)) {
	  duplicate = true;
	  break;
	}
      }
      if(duplicate) continue;

      newtris.push_back(tri);
      incidentTris[tri.a].push_back(newtris.size()-1);
      incidentTris[tri.b].push_back(newtris.size()-1);
      incidentTris[tri.c].push_back(newtris.size()-1);
    }
  }
  printf("Deleted %d triangles\n",mesh.tris.size()-newtris.size());
  mesh.tris = newtris;
}

int main(int argc, char** argv)
{
  if(argc < 2) {
    printf(USAGE_STRING);
    printf(OPTIONS_STRING);
    return 0;
  }
  RobotWorld world;
  world.lights.resize(1);
  world.lights[0].setColor(GLColor(1,1,1));
  world.lights[0].setDirectionalLight(Vector3(0.2,-0.4,1));
  world.lights[0].setColor(GLColor(1,1,1));
  RobotPoseProgram program(&world);
  if(!program.LoadCommandLine(argc,argv)) return 1;

  //debugging limb shrinking for yajia
  /*
  double threshold = -0.015;
  Array3D<double> distance;
  distance.resize( 40, 40, 40);
  Array3D<Vector3> gradient;

  Robot* robot = world.robots[0].robot;
  AABB3D bb;

  printf("Computing mesh shrunk by 1.5cm\n");
  vector<IntTriple> surfaceCells;
  Geometry::CollisionMesh* mesh = &robot->geometry[60];
  mesh->CalcIncidentTris();
  mesh->CalcTriNeighbors();
  FastMarchingMethod_Fill( *mesh, distance, gradient, bb, surfaceCells);
  
  MarchingCubes( distance, threshold, bb, *mesh);
  CollapseVerts(*mesh);
  ofstream fn("newmesh.tri",ios::out);
  fn<<*mesh<<endl;
  fn.close();
  */

  return program.Run();
}
