#ifndef VIEW_RIGID_OBJECT_H
#define VIEW_RIGID_OBJECT_H

#include "Modeling/RigidObject.h"
#include <GLdraw/GLColor.h>
using namespace GLDraw;

/** @ingroup View 
 * @brief Displays an RigidObject structure using OpenGL.
 */
struct ViewRigidObject
{
  ViewRigidObject();
  void Draw(RigidObject* _obj) { obj=_obj; Draw(); }
  void Draw();

  RigidObject* obj;
  GLColor faceColor,edgeColor;
  bool lighting;
  bool edges;
};


#endif
