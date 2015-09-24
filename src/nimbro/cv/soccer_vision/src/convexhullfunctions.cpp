// Soccer Vision Node
// Author: Julio Pastrana <pastrana@ais.uni-bonn.de>

#include "convexhullfunctions.h"

using namespace std;
using namespace soccervision;

//...........................................................................
// 2D cross product of OA and OB vectors, i.e. z-component of their 3D cross product.
// Returns a positive value, if OAB makes a counter-clockwise turn,
// negative for clockwise turn, and zero if the HullPoints are collinear.
long long soccervision::cross(const HullPoint &O, const HullPoint &A, const HullPoint &B)
{
        return (A.x - O.x) * (B.y - O.y) - (A.y - O.y) * (B.x - O.x);
}
//...........................................................................


//...........................................................................
// 2D cross product of OA and OB vectors, i.e. z-component of their 3D cross product.
// Returns a positive value, if OAB makes a counter-clockwise turn,
// negative for clockwise turn, and zero if the HullPoints are collinear.
long long soccervision::cross(const PixelPosition &O, const PixelPosition &A, const PixelPosition &B)
{
        return (A.x - O.x) * (B.y - O.y) - (A.y - O.y) * (B.x - O.x);
}
//...........................................................................

//...........................................................................
//...........................................................................
// Implementation of monotone chain 2D convex hull algorithm.
// Asymptotic complexity: O(n log n).


// Returns a list of HullPoints on the convex hull in counter-clockwise order.
// Note: the last HullPoint in the returned list is the same as the first one.
vector<HullPoint> soccervision::convex_hull(vector<HullPoint> P)
{
        int n = P.size(), k = 0;
        vector<HullPoint> H(2*n);

        // Sort HullPoints lexicographically
        sort(P.begin(), P.end());

        // Build lower hull
        for (int i = 0; i < n; i++) {
                while (k >= 2 && cross(H[k-2], H[k-1], P[i]) <= 0) k--;
                H[k++] = P[i];
        }

        // Build upper hull
        for (int i = n-2, t = k+1; i >= 0; i--) {
                while (k >= t && cross(H[k-2], H[k-1], P[i]) <= 0) k--;
                H[k++] = P[i];
        }

        H.resize(k);
        return H;
}
//...........................................................................

vector<HullPoint> soccervision::convex_hull_on_array(HullPoint P[], int size)
{
	int n = size, k = 0;	
	HullPoint H[2*SUB_SAMPLING_WIDTH*SUB_SAMPLING_HEIGHT];

        // Sort points lexicographically
        sort(P, P+size);

        // Build lower hull
        for (int i = 0; i < n; i++) {
                while (k >= 2 && cross(H[k-2], H[k-1], P[i]) <= 0) k--;
                H[k++] = P[i];
        }

        // Build upper hull

        for (int i = n-2, t = k+1; i >= 0; i--) {
                while (k >= t && cross(H[k-2], H[k-1], P[i]) <= 0) k--;
                H[k++] = P[i];
        }
       
        vector<HullPoint> R;
		for (int p=0; p<k;p++){
			R.push_back(H[p]);
		}
	
        return R;
}
//...........................................................................


//...........................................................................

void soccervision::convex_hull_on_fixed_array(HullPoint P[], int size, regionStack & R)
{
	int n = size, k = 0;	
	R.clear();
	
	HullPoint H[2*SUB_SAMPLING_WIDTH*SUB_SAMPLING_HEIGHT];

        // Sort points lexicographically
        sort(P, P+size);

        // Build lower hull
        for (int i = 0; i < n; i++) {
                while (k >= 2 && cross(H[k-2], H[k-1], P[i]) <= 0) k--;
                H[k++] = P[i];
        }

        // Build upper hull

        for (int i = n-2, t = k+1; i >= 0; i--) {
                while (k >= t && cross(H[k-2], H[k-1], P[i]) <= 0) k--;
                H[k++] = P[i];
        }
       
        PixelPosition pix;
		for (int p=0; p<k;p++){
			pix.x = H[p].x;
			pix.y = H[p].y;
			R.push(pix);
		}
	
}
//...........................................................................

//...........................................................................

void soccervision::convex_hull_on_fixed_pixelregion_array(regionStack &RS, regionStack & R)
{
	
	int n = RS.size(), k = 0;	


	PixelPosition *P = RS.arr;
	PixelPosition H[2*SUB_SAMPLING_WIDTH*SUB_SAMPLING_HEIGHT];
	
	// Sort points lexicographically
	sort(P, P + n);

	// Build lower hull
	for (int i = 0; i < n; i++) {
			while (k >= 2 && cross(H[k-2], H[k-1], P[i]) <= 0) k--;
			H[k++] = P[i];
	}

	// Build upper hull
	for (int i = n-2, t = k+1; i >= 0; i--) {
			while (k >= t && cross(H[k-2], H[k-1], P[i]) <= 0) k--;
			H[k++] = P[i];
	}
	
	
	PixelPosition pix;
	R.clear();
	for (int p=0; p<k;p++){
		pix.x = H[p].x;
		pix.y = H[p].y;
		R.push(pix);
	}
	
}
//...........................................................................


//...........................................................................
// Implementation of line generation using Bresenham's algorithm
void soccervision::lineBresenham_on_stack(int x0, int y0, int x1, int y1,  regionStack & line)
{
 
    PixelPosition tmpPoint;

    int dy = y1 - y0;
    int dx = x1 - x0;
    int stepx, stepy;

    if (dy < 0) { dy = -dy;  stepy = -1; } else { stepy = 1; }
    if (dx < 0) { dx = -dx;  stepx = -1; } else { stepx = 1; }
    dy <<= 1;                                                  // dy is now 2*dy
    dx <<= 1;                                                  // dx is now 2*dx

    tmpPoint.x = x0;
    tmpPoint.y = y0;
    line.push(tmpPoint);

	
    if (dx > dy) {
        int fraction = dy - (dx >> 1);                         // same as 2*dy - dx
        while (x0 != x1) {
            if (fraction >= 0) {
                y0 += stepy;
                fraction -= dx;                                // same as fraction -= 2*dx
            }
            x0 += stepx;
            fraction += dy;                                    // same as fraction -= 2*dy


			tmpPoint.x = x0;
			tmpPoint.y = y0;
			line.push(tmpPoint);
			
        }
    } else {
        int fraction = dx - (dy >> 1);
        while (y0 != y1) {
            if (fraction >= 0) {
                x0 += stepx;
                fraction -= dy;
            }
            y0 += stepy;
            fraction += dx;

			tmpPoint.x = x0;
			tmpPoint.y = y0;
			line.push(tmpPoint);
        }
    }
}
//...........................................................................




//...........................................................................
// Implementation of line generation using Bresenham's algorithm
void soccervision::lineBresenham(int x0, int y0, int x1, int y1, vector <HullPoint> & puntos)
{
 
    HullPoint tmpPoint;

    int dy = y1 - y0;
    int dx = x1 - x0;
    int stepx, stepy;

    if (dy < 0) { dy = -dy;  stepy = -1; } else { stepy = 1; }
    if (dx < 0) { dx = -dx;  stepx = -1; } else { stepx = 1; }
    dy <<= 1;                                                  // dy is now 2*dy
    dx <<= 1;                                                  // dx is now 2*dx

    tmpPoint.x = x0;
    tmpPoint.y = y0;
    puntos.push_back(tmpPoint);

    if (dx > dy) {
        int fraction = dy - (dx >> 1);                         // same as 2*dy - dx
        while (x0 != x1) {
            if (fraction >= 0) {
                y0 += stepy;
                fraction -= dx;                                // same as fraction -= 2*dx
            }
            x0 += stepx;
            fraction += dy;                                    // same as fraction -= 2*dy


            tmpPoint.x = x0;
            tmpPoint.y = y0;
            puntos.push_back(tmpPoint);

        }
    } else {
        int fraction = dx - (dy >> 1);
        while (y0 != y1) {
            if (fraction >= 0) {
                x0 += stepx;
                fraction -= dy;
            }
            y0 += stepy;
            fraction += dx;

            tmpPoint.x = x0;
            tmpPoint.y = y0;
            puntos.push_back(tmpPoint);
   
        }
    }
}
//...........................................................................



//This function tells if a line is within certain white area
//using Bresenham's algorithm to generate the line.
//Note that the image that gets to this function is one of the subsampled ones.
bool soccervision::isLineOnImage(int x0, int y0, int x1, int y1, unsigned char image[])
{
 


	int holeCounter = 0;
	
    int dy = y1 - y0;
    int dx = x1 - x0;
    int stepx, stepy;

    if (dy < 0) { dy = -dy;  stepy = -1; } else { stepy = 1; }
    if (dx < 0) { dx = -dx;  stepx = -1; } else { stepx = 1; }
    dy <<= 1;                                                  // dy is now 2*dy
    dx <<= 1;                                                  // dx is now 2*dx


	int pixelpos = (SUB_SAMPLING_WIDTH+10)*(y0) + (x0);
	if ( image[pixelpos] == 0) { holeCounter ++; }
    
    
    if (dx > dy) {
        int fraction = dy - (dx >> 1);                         // same as 2*dy - dx
        while (x0 != x1) {
            if (fraction >= 0) {
                y0 += stepy;
                fraction -= dx;                                // same as fraction -= 2*dx
            }
            x0 += stepx;
            fraction += dy;                                    // same as fraction -= 2*dy

			int pixelpos = (SUB_SAMPLING_WIDTH+10)*(y0) + (x0);
			if ( image[pixelpos] == 0) { holeCounter ++; }
            
        }
    } else {
        int fraction = dx - (dy >> 1);
        while (y0 != y1) {
            if (fraction >= 0) {
                x0 += stepx;
                fraction -= dy;
            }
            y0 += stepy;
            fraction += dx;

			int pixelpos = (SUB_SAMPLING_WIDTH+10)*(y0) + (x0);
			if ( image[pixelpos] == 0) { holeCounter ++; }
        }
    }
    
    if (holeCounter<=1)
		return true;
	else
		return false;
}
//...........................................................................
