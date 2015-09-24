#ifndef __LINE_FWD_H__
#define __LINE_FWD_H__
#include "nimbroStyleTools.h"

namespace ObjectRecognition{
	struct FieldLine2 {
		float len;
		Vec2i s, e;
		float ang;
		float conf;
		float curvature;
		Vec2i s_world, e_world;
		Vec2i lot_world;
		WorldObjectType wo;
		int num;
		void* mNodes;
		FieldLine2( float l, Vec2i & _s, Vec2i & _e, Vec2i & _ws, Vec2i & _we ) 
			: len( l )
			, s( _s )
			, e( _e )
			, conf( 1.f )
			, s_world( _ws )
			, e_world( _we )
			, num( 0 ) 
		{ }

		FieldLine2( ) 
			: len(0.1f)
			, num( 0 ) 
		{ }
	};
	class Line;
};

#endif