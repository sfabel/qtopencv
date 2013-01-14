//This is a library of 3D Robot rendering, it contains several tools.
#include "GRtool.h"


//=====================Draw Solid Cylinder=========================
void grDrawSolidCylinder(GLfloat baseRadius, GLfloat topRadius, GLfloat Height)
{
	GLUquadricObj *pObj;	// Temporary, used for quadrics
		
	// Setup the quadric object
	pObj = gluNewQuadric();
	glPushMatrix();
        glTranslatef(0.0f, 0.0f, -Height/2);
	    //gluCylinder (GLUquadric* quad, GLfloat base, GLfloat top, GLfloat height, GLint slices, GLint stacks);
	    gluCylinder(pObj, baseRadius, topRadius, Height, 20, 4);
	    //Draw a cone as top surface
	    glutSolidCone(baseRadius, Height/2, 20, 1);
	    //Draw a cone as  bottom surface
        glPushMatrix();
	        glTranslatef(0.0f,  0.0f, Height);
            glRotatef(180, 1.0f, 0.0f, 0.0f);
	        glutSolidCone(topRadius, Height/2, 20, 1);
        glPopMatrix();
    glPopMatrix();
}

//=====================Draw Rotational Joint=========================
void grDrawRJoint(GLfloat baseRadius, GLfloat topRadius, GLfloat Height, GLfloat _R, GLfloat _G, GLfloat _B, GLfloat _Alpha)
{
        glColor4f(_R, _G, _B, _Alpha); //Red
	    grDrawSolidCylinder(baseRadius, topRadius, Height); //Draw a solid closed Cylinder
        //Then draw three flanges for locomotion identification
		glColor4f((_R+_G)*1000, (_R+_G)*1000, 0.0f, _Alpha); //yellow
		glPushMatrix();
		   //first flange
			glPushMatrix();			   
			    glTranslatef(0.0f, baseRadius/2+0.1f*baseRadius, 0.0f);
			    glScalef(0.1*baseRadius, 0.8f*baseRadius, Height*1.1);
	            glutSolidCube(1.0f); 
			glPopMatrix();
           //second flange
			glRotatef(120, 0.0f, 0.0f, 1.0f);
			glPushMatrix(); 
			    glTranslatef(0.0f, baseRadius/2+0.1f*baseRadius, 0.0f);
			    glScalef(0.1*baseRadius, 0.8f*baseRadius, Height*1.1);
	            glutSolidCube(1.0f);
			glPopMatrix();
		   //third flange
		    glRotatef(120, 0.0f, 0.0f, 1.0f);
			glPushMatrix();
			    glTranslatef(0.0f, baseRadius/2+0.1f*baseRadius, 0.0f);
			    glScalef(0.1*baseRadius, 0.8f*baseRadius, Height*1.1);
	            glutSolidCube(1.0f); 
			glPopMatrix();
		glPopMatrix();
}

//===================Draw Base====================
//Draw a Base of robot, which is a solid cylinder + a flat box
void grDrawBase(GLfloat baseRadius, GLfloat Height, GLfloat R, GLfloat G, GLfloat B, GLfloat ALPHA)
{
	glColor4f(R,  G, B, ALPHA); //gray   
	     glPushMatrix();
		        //Draw a solid cylinder as base
                grDrawSolidCylinder(baseRadius, baseRadius, 0.9f*Height);
                //Draw a flat box as pedesta
			    glTranslatef(0.0f, 0.0f, -Height/2);
			    glScalef(2.2f*baseRadius, 2.2f*Height, 0.1f*baseRadius);
	            glutSolidCube(1.0f); 
		 glPopMatrix(); 	   
        
}

//===============Draw Prismatic Joint==============================
void grDrawPJoint(GLfloat baseRadius, GLfloat topRadius, GLfloat Length, GLfloat _R, GLfloat _G, GLfloat _B, GLfloat _Alpha)
{
	const GLfloat Ratio = 0.4f; //sleeve ration of the total length
    glPushMatrix();
	    glTranslatef(0.0f, 0.0f, -Length); //Let the head of prismatic joint align with the origin of prismatic Frame.
		//--------------------------------------Sleeve
	    glPushMatrix();
	         glTranslatef(0.0f, 0.0f, Length*Ratio*0.5f);
	         glScalef(baseRadius*2.0f, baseRadius*2.0f, Length*Ratio); //draw sleeve of prismatic joint, which is 10% of total length
			   	glColor4f(_R*0.7f, _G, _B, _Alpha); //dark Red 
             	glutSolidCube(1.0f); 	
			    glColor4f(0.0f, 0.0f, 0.0f, 1.0f); //draw black wire
		        glutWireCube(1.0f);   
	    glPopMatrix();	 
	   //---------------------------------------Arm
	   
	    glPushMatrix();
	        glTranslatef(0.0f, 0.0f, Length*(0.5f+0.5*Ratio));
	        glScalef(topRadius*2.0f, topRadius*2.0f, Length*(1-Ratio)); //draw arm of prismatic joint, which is 90% of total length
			glColor4f(_R, _G, _B, _Alpha); //Red
            glutSolidCube(1.0f);
			glColor4f(0.0f, 0.0f, 0.0f, 1.0f); //draw black wire
		    glutWireCube(1.0f);   
        glPopMatrix();

	glPopMatrix();

}

//void grDrawSolidBox(GLfloat x, GLfloat y, GLfloat z, GLfloat Radius, GLfloat R, GLfloat G, GLfloat B, GLfloat ALPHA) //Draw a box from origin to (x,y,z);
//{
//  const float PI=3.1415926;
//  GLfloat m[16];
//  GLdouble alpha, theta;
//  GLfloat Length, Ri;
//   //calculate rotation angle: alpha and theta
//    Length = sqrt(x*x+y*y+z*z);
//	Ri = sqrt(x*x+y*y);
//	theta = 90+ atan2(y, x)*180.0/PI; //degrees
//    alpha = atan2(Ri, z)*180.0/PI; //degrees

//	glColor4f(R, G, B, ALPHA);
//   //generate GL-compatible rotation matrix
//	grRotMatrix(alpha, theta, m);

//  	glPushMatrix();
//		 glMatrixMode(GL_MODELVIEW);
//         //glMultMatrixf(m);
//		 glRotatef(theta,0.0f, 0.0f, 1.0f);
//		 glRotatef(alpha, 1.0f, 0.0f, 0.0f);
//	     glTranslatef(0.0f, 0.0f, Length*0.5f);
//	     glScalef(Radius, Radius, Length);
//         glutSolidCube(1.0f);
//		 glColor4f(0.0f, 0.0f, 0.0f, 1.0f);
//		 glutWireCube(1.0f);
//    glPopMatrix();
//}

void grDrawEndEffector(GLfloat size, GLint mode, GLfloat R, GLfloat G, GLfloat B, GLfloat ALPHA)
{   //change mode, change oritation of end effector to x+, x-, y+, y-, z+ and z- accroding to 'mode'
        glPushMatrix();
            //Change mode
        switch (mode) {
                case 1: //x+
                        glRotatef(90, 0.0f, 1.0f, 0.0f);
                        break;
                case 2: //x-
                        glRotatef(-90, 0.0f, 1.0f, 0.0f);
                        break;
                case 3: //y+
                        glRotatef(90, 1.0f, 0.0f, 0.0f);
                        break;
                case 4: //y-
                        glRotatef(-90, 1.0f, 0.0f, 0.0f);
                        break;
                case 5: //z-
                        glRotatef(180, 1.0f, 0.0f, 0.0f);
                        break;
                case 6:
                        break;
                default:
                        //z+
                        break;
        }

                        //Draw a base of grasper
			
                        glPushMatrix();
                                glScalef(size, 0.5*size, 0.1*size);
                                glColor4f(R, G, B, ALPHA);
                                glutSolidCube(1.0f);
                            glColor4f(0.0f, 0.0f, 0.0f, 1.0f); //draw black wire
                        glutWireCube(1.0f);
                        glPopMatrix();
                         //Draw finger 1
                        glPushMatrix();
                                glTranslatef(0.35*size, 0.0f, 0.2*size);
                                glRotatef(-30, 0.0f, 1.0f, 0.0f);
                                glScalef(0.1*size, 0.5*size, 0.6*size);
                                glColor4f(R, G, B, ALPHA);
                                glutSolidCube(1.0f);
                            glColor4f(0.0f, 0.0f, 0.0f, 1.0f); //draw black wire
                        glutWireCube(1.0f);
                        glPopMatrix();
                        //Draw finger 2
                        glPushMatrix();
                                glTranslatef(-0.35*size, 0.0f, 0.2*size);
                                glRotatef(30, 0.0f, 1.0f, 0.0f);
                                glScalef(0.1*size, 0.5*size, 0.6*size);
                                glColor4f(R, G, B, ALPHA);
                                glutSolidCube(1.0f);
                            glColor4f(0.0f, 0.0f, 0.0f, 1.0f); //draw black wire
                        glutWireCube(1.0f);
                        glPopMatrix();
        glPopMatrix();
}

void grDrawGround(GLfloat Length, GLfloat Width, GLfloat zPosition)
{
            glPushMatrix();
                   glRotatef(90, 1.0f, 0.0f, 0.0f);
                  // Draw the ground; we do manual shading to a darker green
                  // in the background to give the illusion of depth
                        glBegin(GL_QUADS);
                        glColor3f(0.1, 0.1, 0.1);
                        glVertex3f(Length, -Width, -zPosition);
                        glVertex3f(-Length, -Width, -zPosition);

                        glVertex3f(-Length, Width, -zPosition);
                        glVertex3f(Length, Width, -zPosition);
                        glEnd();
                glPopMatrix();
}
void grBeginShadow(GLfloat zPosition, GLfloat *lightPos)
{
    M3DMatrix44f shadowMat;
        // Any three points on the ground (counter clockwise order)
    M3DVector3f points[3] = {{ 1.0f,  1.0f, zPosition},
                            { 40.0f, 15.0f, zPosition},
                            { 15.0f, 40.0f,  zPosition}};
        // Get the plane equation from three points on the ground
    M3DVector4f vPlaneEquation;
    m3dGetPlaneEquation(vPlaneEquation, points[0], points[1], points[2]);
    // Calculate projection matrix to draw shadow on the ground
    m3dMakePlanarShadowMatrix(shadowMat, vPlaneEquation, lightPos);
    glPushMatrix();
                //glRotatef(90, 1.0f, 0.0f, 0.0f);
            // Get ready to draw the shadow and the ground
                // First disable lighting and save the projection state
                glDisable(GL_DEPTH_TEST);
                glDisable(GL_LIGHTING);
                // Multiply by shadow projection matrix
                glMultMatrixf((GLfloat *)shadowMat);
}

void grEndShadow()
{
        glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
        glPopMatrix();
}


//void grHomoMatrix(float alpha, float a, float d, float theta, float *m) //degrees
//{
//        const float PI= 3.1415926;
//        alpha = alpha*PI/180.0; //rad
//        theta = theta*PI/180.0; //rad

//      *m = cos(theta);
//          *(m+1) = sin(theta)*cos(alpha);
//          *(m+2) = sin(theta)*sin(alpha);
//          *(m+3) = 0.0;

//          *(m+4) = -sin(theta);
//          *(m+5) = cos(theta)*cos(alpha);
//          *(m+6) = cos(theta)*sin(alpha);
//          *(m+7) = 0.0;

//          *(m+8) = 0.0;
//          *(m+9) = -sin(alpha);
//          *(m+10) = cos(alpha);
//          *(m+11) = 0.0;

//          *(m+12) = a;
//          *(m+13) = -sin(alpha)*d;
//          *(m+14) = cos(alpha)*d;
//          *(m+15) = 1.0;
	  
//}

//void grRotMatrix(float alpha, float theta, float *m) //degrees
//{
//           const float PI= 3.1415926;
//       alpha = alpha*PI/180.0; //rad
//           theta = theta*PI/180.0; //rad

//      *m = cos(theta);
//          *(m+1) = sin(theta)*cos(alpha);
//          *(m+2) = sin(theta)*sin(alpha);
//          *(m+3) = 0.0;

//          *(m+4) = -sin(theta);
//          *(m+5) = cos(theta)*cos(alpha);
//          *(m+6) = cos(theta)*sin(alpha);
//          *(m+7) = 0.0;

//          *(m+8) = 0.0;
//          *(m+9) = -sin(alpha);
//          *(m+10) = cos(alpha);
//          *(m+11) = 0.0;

//          *(m+12) = 0.0;
//          *(m+13) = 0.0;
//          *(m+14) = 0.0;
//          *(m+15) = 1.0;
	  
//}
