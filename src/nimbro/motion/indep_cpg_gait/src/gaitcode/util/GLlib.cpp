#include "GLlib.h"
#include <GL/Glu.h>
#include <math.h>

namespace indep_cpg_gait
{
	namespace GLlib
	{
		// 1: top left front
		// 2: bottom left front
		// 3: top right front
		// 4: bottom right front
		// 5: top right back
		// 6: bottom right back
		// 7: top left back
		// 8: bottom left back
		void drawBox(
				double x1, double y1, double z1,
				double x2, double y2, double z2,
				double x3, double y3, double z3,
				double x4, double y4, double z4,
				double x5, double y5, double z5,
				double x6, double y6, double z6,
				double x7, double y7, double z7,
				double x8, double y8, double z8
				)
		{
			glBegin( GL_QUAD_STRIP );
			glVertex3f(x1, y1, z1);
			glVertex3f(x2, y2, z2);
			glVertex3f(x3, y3, z3);
			glVertex3f(x4, y4, z4);
			glVertex3f(x5, y5, z5);
			glVertex3f(x6, y6, z6);
			glVertex3f(x7, y7, z7);
			glVertex3f(x8, y8, z8);
			glVertex3f(x1, y1, z1);
			glVertex3f(x2, y2, z2);
			glEnd();

			glBegin( GL_QUADS );
			glVertex3f(x1, y1, z1);
			glVertex3f(x3, y3, z3);
			glVertex3f(x5, y5, z5);
			glVertex3f(x7, y7, z7);
			glVertex3f(x2, y2, z2);
			glVertex3f(x4, y4, z4);
			glVertex3f(x6, y6, z6);
			glVertex3f(x8, y8, z8);
			glEnd();

			glColor3f(0.3, 0.3, 0.3);
			glLineWidth(2);
			glBegin( GL_LINE_STRIP );
			glVertex3f(x1, y1, z1);
			glVertex3f(x2, y2, z2);
			glVertex3f(x4, y4, z4);
			glVertex3f(x3, y3, z3);
			glVertex3f(x1, y1, z1);
			glVertex3f(x7, y7, z7);
			glVertex3f(x8, y8, z8);
			glVertex3f(x6, y6, z6);
			glVertex3f(x5, y5, z5);
			glVertex3f(x7, y7, z7);
			glEnd();

			glBegin( GL_LINES );
			glVertex3f(x3, y3, z3);
			glVertex3f(x5, y5, z5);
			glVertex3f(x4, y4, z4);
			glVertex3f(x6, y6, z6);
			glVertex3f(x2, y2, z2);
			glVertex3f(x8, y8, z8);
			glEnd();
		}

		// Draws a cuboid with half extents x, y and z.
		void drawBox(float x, float y, float z)
		{
			glBegin( GL_QUAD_STRIP );
			glVertex3f(x, y, z);
			glVertex3f(x, y, -z);
			glVertex3f(x, -y, z);
			glVertex3f(x, -y, -z);
			glVertex3f(-x, -y, z);
			glVertex3f(-x, -y, -z);
			glVertex3f(-x, y, z);
			glVertex3f(-x, y, -z);
			glVertex3f(x, y, z);
			glVertex3f(x, y, -z);
			glEnd();

			glBegin( GL_QUADS );
			glVertex3f(x, y, z);
			glVertex3f(x, -y, z);
			glVertex3f(-x, -y, z);
			glVertex3f(-x, y, z);
			glVertex3f(x, y, -z);
			glVertex3f(x, -y, -z);
			glVertex3f(-x, -y, -z);
			glVertex3f(-x, y, -z);
			glEnd();

			glColor3f(0.3, 0.3, 0.3);
			glLineWidth(2);
			glBegin( GL_LINE_STRIP );
			glVertex3f(x, y, z);
			glVertex3f(x, y, -z);
			glVertex3f(x, -y, -z);
			glVertex3f(x, -y, z);
			glVertex3f(x, y, z);
			glVertex3f(-x, y, z);
			glVertex3f(-x, y, -z);
			glVertex3f(-x, -y, -z);
			glVertex3f(-x, -y, z);
			glVertex3f(-x, y, z);
			glEnd();

			glBegin( GL_LINES );
			glVertex3f(x, -y, z);
			glVertex3f(-x, -y, z);
			glVertex3f(x, -y, -z);
			glVertex3f(-x, -y, -z);
			glVertex3f(x, y, -z);
			glVertex3f(-x, y, -z);
			glEnd();
		}

		// Draws a sphere.
		void drawSphere(float radius)
		{
			static GLUquadric* quadric = gluNewQuadric();
			gluSphere(quadric, radius, 32, 32);
		}

		// Draws a sphere with a camera plane aligned border around it.
		void drawBorderedSphere(float radius)
		{
			static GLUquadric* quadric = gluNewQuadric();
			gluSphere(quadric, radius, 32, 32);
		}

		// Draws a circle in the xy plane.
		// http://slabode.exofire.net/circle_draw.shtml
		void drawCircle(float radius, int slices)
		{
			float theta = 2 * 3.1415926 / float(slices);
			float c = cos(theta); //precalculate the sine and cosine
			float s = sin(theta);
			float t;

			float x = radius;//we start at angle = 0
			float y = 0;

			glBegin(GL_LINE_LOOP);
			for(int ii = 0; ii < slices; ii++)
			{
				glVertex3f(x, y, 0);//output vertex

				//apply the rotation matrix
				t = x;
				x = c * x - s * y;
				y = s * t + c * y;
			}
			glEnd();
		}

		// Draws a coordinate frame.
		void drawFrame(float size)
		{
			glLineWidth(2);
			glBegin( GL_LINES );
			glColor3f(1.0, 0.0, 0.0);
			glVertex3f(0.0f, 0.0f, 0.0f);
			glVertex3f(size, 0.0f, 0.0f);
			glColor3f(0.0, 1.0, 0.0);
			glVertex3f(0.0f, 0.0f, 0.0f);
			glVertex3f(0.0f, size, 0.0f);
			glColor3f(0.0, 0.0, 1.0);
			glVertex3f(0.0f, 0.0f, 0.0f);
			glVertex3f(0.0f, 0.0f, size);
			glEnd();
		}
	};
}