#include "Environment.h"
#include <meshing/IO.h>
#include <utils/SimpleFile.h>
#include <utils/stringutils.h>
#include <string.h>
#include <fstream>

bool Environment::Load(const char* fn)
{
  const char* ext=FileExtension(fn);
  if(0==strcmp(ext,"env")) {
    SimpleFile f(fn);
    if(!f) {
      fprintf(stderr,"SimpleFile read failed\n");
      return false;
    }
    if(f.count("mesh")==0) {
      fprintf(stderr,"Environment file doesn't contain a mesh file\n");
      return false;
    }
    if(!f.CheckSize("mesh",1,fn)) return false;
    string fnPath = GetFilePath(fn);
    string meshfn = fnPath + f["mesh"][0].AsString();
    if(!Meshing::Import(meshfn.c_str(),mesh)) {
      cout<<"Environment::Load error loading "<<meshfn<<endl;
      return false;
    }
    mesh.InitCollisions();
    if(f.count("kFriction")!=0) {
      if(!f.CheckType("kFriction",PrimitiveValue::Double,fn)) return false;
      vector<double> values=f.AsDouble("kFriction");
      if(values.size() == 1)
	SetUniformFriction(values[0]);
      else if(values.size() == mesh.tris.size()) {
	kFriction = values;
      }
      else {
	fprintf(stderr,"Environment file doesn't contain the right number of friction values\n");
	return false;
      }
    }
    return true;
  }
  else if(Meshing::CanLoadTriMeshExt(ext)) {
    if(!Meshing::Import(fn,mesh)) return false;
    mesh.InitCollisions();
    SetUniformFriction(0.5);
    return true;
  }
}

//bool Environment::Save(const char* fn);
