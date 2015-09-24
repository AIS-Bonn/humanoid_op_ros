// Image array algorithms
// Extracted by Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef IMGARRAYTOOLS_H
#define IMGARRAYTOOLS_H

double abs(double x)
{
	if(x < 0)
		return -x;
	return x;
}

// taken from http://www.wol.net.pk/mtshome/cppComputerGraphics.html#Line
// and modified for callback
// Idea: Track along line (x1,y1)-(x2,y2) and call the CallBack object with the coordinates
//   stop if callBack returns false
template < class T >
bool
bresenham( T & callBack, const Vec2f a, const Vec2f b, int xres, int yres, float xoff, float yoff, float gxoff, float gyoff )
{
	return bresenham( callBack, ( int ) ( ( a.x() + xoff ) / xres + gxoff ), ( int ) ( ( a.y() + yoff ) / yres + gyoff ), ( int ) ( ( b.x() + xoff ) / xres + gxoff ), ( int ) ( ( b.y() + yoff ) / yres + gyoff ) );
}

template < class T >
bool
bresenham( T & callBack, const Vec2i a, const Vec2i b )
{
	return bresenham( callBack, a.x, a.y, b.x, b.y );
}

template < class T >
bool
bresenham( T & callBack, const int x_1, const int y_1, const int x_2, const int y_2 )
{
	int x1 = x_1;
	int y1 = y_1;
	int x2 = x_2;
	int y2 = y_2;

	if ( x_1 > x_2 ) {
		x1 = x_2;
		y1 = y_2;
		x2 = x_1;
		y2 = y_1;
	}
	int dx = abs( x2 - x1 );
	int dy = abs( y2 - y1 );
	int inc_dec = ( ( y2 >= y1 ) ? 1 : -1 );

	if ( dx > dy ) {
		int two_dy = ( 2 * dy );
		int two_dy_dx = ( 2 * ( dy - dx ) );
		int p = ( ( 2 * dy ) - dx );
		int x = x1;
		int y = y1;

		if ( !callBack( x, y ) ) {
			return false;
		}
		while ( x < x2 ) {
			x++;
			if ( p < 0 ) {
				p += two_dy;
			}
			else {
				y += inc_dec;
				p += two_dy_dx;
			}
			if ( !callBack( x, y ) ) {
				return false;
			}
		}
	}
	else {
		int two_dx = ( 2 * dx );
		int two_dx_dy = ( 2 * ( dx - dy ) );
		int p = ( ( 2 * dx ) - dy );

		int x = x1;
		int y = y1;

		if ( !callBack( x, y ) ) {
			return false;
		}
		while ( y != y2 ) {
			y += inc_dec;
			if ( p < 0 ) {
				p += two_dx;
			}
			else {
				x++;
				p += two_dx_dy;
			}
			if ( !callBack( x, y ) ) {
				return false;
			}
		}
	}
	return true;
}

template < class T >
bool
bresenham_circle( T & callBack, const float h, const float k, const float r )
{
	return bresenham_circle( callBack, ( int ) h, ( int ) k, ( int ) r );
}

template < class T >
bool
bresenham_circle( T & callBack, const int h, const int k, const int r )
{
	int x = 0;
	int y = r;
	int p = ( 3 - ( 2 * r ) );

	do {
		//if(!callBack((h+x),(k+y))) return false;
		//if(!callBack((h+y),(k+x))) return false;
		//if(!callBack((h+y),(k-x))) return false;
		//if(!callBack((h+x),(k-y))) return false;
		//if(!callBack((h-x),(k-y))) return false;
		//if(!callBack((h-y),(k-x))) return false;
		//if(!callBack((h-y),(k+x))) return false;
		//if(!callBack((h-x),(k+y))) return false;

		if ( !callBack( ( h + x ), ( k + y ) ) ) {
			return false;
		}
		if ( !callBack( ( h + y ), ( k + x ) ) ) {
			return false;
		}
		if ( !callBack( ( h + y ), ( k - x ) ) ) {
			return false;
		}
		if ( !callBack( ( h + x ), ( k - y ) ) ) {
			return false;
		}
		if ( !callBack( ( h - x ), ( k - y ) ) ) {
			return false;
		}
		if ( !callBack( ( h - y ), ( k - x ) ) ) {
			return false;
		}
		if ( !callBack( ( h - y ), ( k + x ) ) ) {
			return false;
		}
		if ( !callBack( ( h - x ), ( k + y ) ) ) {
			return false;
		}
		x++;

		if ( p < 0 ) {
			p += ( ( 4 * x ) + 6 );
		}
		else {
			y--;
			p += ( ( 4 * ( x - y ) ) + 10 );
		}
	} while ( x <= y );
	return true;
}

#endif
