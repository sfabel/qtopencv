#ifndef _GRTOOL_H__
#define _GRTOOL_H__

//This is the header file of GRtool.cpp
#include <GL/freeglut.h>
#include "math3d.h"
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glx.h>

const int MAX_LINK = 17;
struct DH_parameter
{
  char  type;   //Rotational, Prismatic
  float alpha; //alpha(i-1)
  float a;       //a(i-1)
  float d;       //d(i)
  float theta;  //theta(i)
  float m[16]; //Homogeneous Matrix, in 1D matrix form, compatible with OpenGL matrix standard
}; 


void grDrawSolidCylinder(GLfloat baseRadius, GLfloat topRadius, GLfloat Height);
void grDrawRJoint(GLfloat baseRadius, GLfloat topRadius, GLfloat Height, GLfloat _R, GLfloat _G, GLfloat _B, GLfloat _Alpha); //Rotational Joint
void grDrawPJoint(GLfloat baseRadius, GLfloat topRadius, GLfloat Length, GLfloat _R, GLfloat _G, GLfloat _B, GLfloat _Alpha); //Prismatic Joint
void grDrawBase(GLfloat baseRadius, GLfloat Height, GLfloat R, GLfloat G, GLfloat B, GLfloat ALPHA);

void grDrawSolidBox(GLfloat x, GLfloat y, GLfloat z, GLfloat Radius, GLfloat R, GLfloat G, GLfloat B, GLfloat ALPHA);//Draw a box from origin to (x,y,z);
void grDrawEndEffector(GLfloat size, GLint mode, GLfloat R, GLfloat G, GLfloat B, GLfloat ALPHA);
void grDrawGround(GLfloat Length, GLfloat Width, GLfloat Height);
void grBeginShadow(GLfloat zPosition, GLfloat *lightPos);
void grEndShadow();

//void grHomoMatrix(float alpha, float a, float d, float theta, float *m);
//void grRotMatrix(float alpha, float theta, float *m);

#endif
