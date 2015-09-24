#ifndef GLLIB
#define GLLIB
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
			);

		// Draws a cuboid with half extents x, y and z.
		void drawBox(float x, float y, float z);

		// Draws a sphere.
		void drawSphere(float radius = 1.0);

		// Draws a sphere with a camera plane aligned border around it.
		void drawBorderedSphere(float radius = 1.0);

		// Draws a circle in the xy plane.
		void drawCircle(float radius = 0.1, int slices = 32);

		// Draws a coordinate frame.
		void drawFrame(float size = 0.1);

	};
}
#endif //GLLIB
