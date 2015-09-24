
#include "fieldgrid2d.h"
#include "imgarraytools.h"

namespace FieldGrid2D {

class MaxFilterApplier{
	float* arr;
	float* filter;
	const int W,H,W2,H2;
	public:
	MaxFilterApplier(int w,int h,int w2,int h2,float* a, float* f)
		: arr(a)
		, filter(f)
		, W(w),H(h),W2(w2),H2(h2)
	{}

	bool operator()(const float cx, const float cy){
		return (*this)((int)cx,(int)cy);
	}
	bool operator()(int cx, int cy){
		/*		if(cy<0)  return true;
		 *		if(cy>=H) return true;
		 *		if(cx<0)  return true;
		 *		if(cx>=W) return true;
		 *		arr[cy*W+cx] = 1;
		 *		return true;
		 **/
		for(int y = 0; y < H2; y++){
			 int ycor = cy + y - (H2-1)/2;
			 if(ycor<0)  continue;
			 if(ycor>=H) continue;
			 for(int x = 0; x < W2; x++){
				 int xcor = cx + x - (W2-1)/2;
				 if(xcor<0)  continue;
				 if(xcor>=W) continue;
				 float& cur_val = arr[ycor*W+xcor];
				 float& new_val = filter[y*W2+x];
				 if(new_val<cur_val) 
					 cur_val = new_val;
			 }
		 }
		 return true;
	}
};


template<class PF>
void ClosestWorldObject::init(std::vector<WorldObjectType>&v,const PF&pf){
	std::vector<WorldObjectType>::iterator it;
	WorldObjectType wo;
	for(int y=0;y<mYdim;y++){
		for(int x=0;x<mXdim;x++){
			Vec2f wpos  = getGridToMetric(Vec2i(x,y));
			it          =  v.begin();
			wo          = *v.begin();
			float mind2 = (wpos-pf.worldObjectPos(wo)).norm2();
			for(;it!=v.end();it++){
				float d2 = (wpos-pf.worldObjectPos(*it)).norm2();
				if(d2<mind2){
					mind2 = d2;
					wo = *it;
				}
			}
			set(Vec2i(x,y),wo);
			assert(get(Vec2i(x,y))>=0);
			assert(get(Vec2i(x,y))<NumWorldObjects);
		}
	}
}
template<class PF>
void DistToWhiteGrid::precomputeLineLikelihoodField(const PF& pf){
	field_model::FieldModel* field = field_model::FieldModel::getInstance();

	double hw = 100 * field->width()/2;
	double hl = 100 * field->length()/2;

	Vec2f cornerBlueLeft(    +hw, -hl);
	Vec2f cornerBlueRight(   -hw, -hl);
	Vec2f cornerYellowLeft(  -hw, +hl);
	Vec2f cornerYellowRight( +hw, +hl);

	Vec2f yellowPoleTXing(   -hw, 0);
	Vec2f bluePoleTXing(     +hw, 0);

	double hgw = 100 * field->goalAreaWidth()/2;
	double gd = 100 * field->goalAreaDepth()/2;
	double circleRadius = 100 * field->centerCircleDiameter();

	// penalty area
	Vec2f strafBlueLeft1 (     +hgw, -hl+gd);
	Vec2f strafBlueLeft2 (     +hgw, -hl);
	Vec2f strafBlueRight1(     -hgw, -hl+gd);
	Vec2f strafBlueRight2(     -hgw, -hl);
	Vec2f strafYellLeft1 (     -hgw, +hl-gd);
	Vec2f strafYellLeft2 (     -hgw, +hl);
	Vec2f strafYellRight1(     +hgw, +hl-gd);
	Vec2f strafYellRight2(     +hgw, +hl);
	Vec2f circlineBlue1  (     -hw,  +circleRadius);
	Vec2f circlineBlue2  (     +hw,  +circleRadius);
	Vec2f circlineYell1  (     -hw,  -circleRadius);
	Vec2f circlineYell2  (     +hw,  -circleRadius);

	for(int i=0;i<mYdim;++i)
		for(int j=0;j<mXdim;++j)
			mGrid[i*mXdim+j] = 1E6;

	static const int filterxres = 81;
	static const int filteryres = 81;

	float gaussfilter[filteryres*filterxres];
	float sigma = 160.f;
	float gscale = gaussian2d(Vec2i(filterxres/2,filteryres/2),filterxres/2.f,filteryres/2.f,sigma/mRes);
	for(int y=0;y<filteryres;y++)
		for(int x=0;x<filterxres;x++)
			//gaussfilter[y*filterxres+x] = 1.f/gscale*RcMath::gaussian2d(Vec2i(x,y),filterxres/2.f,filteryres/2.f,sigma/mRes);
			gaussfilter[y*filterxres+x] = Vec2f((x-filterxres/2.f)*mRes,(y-filteryres/2.f)*mRes).norm()/100.f;

	MaxFilterApplier mfa(mXdim, mYdim,filterxres,filteryres,mGrid,&(gaussfilter[0]));

	float xoff = mXoff, yoff=mYoff;
	float gxoff=(float)mGridXoff,gyoff=(float)mGridYoff;

	// field border
	bresenham(mfa,cornerBlueLeft,cornerBlueRight,mRes,mRes,xoff,yoff,gxoff,gyoff);
	bresenham(mfa,cornerBlueLeft,cornerYellowRight,mRes,mRes,xoff,yoff,gxoff,gyoff);
	bresenham(mfa,cornerYellowLeft,cornerYellowRight,mRes,mRes,xoff,yoff,gxoff,gyoff);
	bresenham(mfa,cornerYellowLeft,cornerBlueRight,mRes,mRes,xoff,yoff,gxoff,gyoff);

	// penalty area
	// ...to field border
	bresenham(mfa,strafBlueLeft1,strafBlueLeft2,mRes,mRes,xoff,yoff,gxoff,gyoff);
	bresenham(mfa,strafBlueRight1,strafBlueRight2,mRes,mRes,xoff,yoff,gxoff,gyoff);
	bresenham(mfa,strafYellLeft1,strafYellLeft2,mRes,mRes,xoff,yoff,gxoff,gyoff);
	bresenham(mfa,strafYellRight1,strafYellRight2,mRes,mRes,xoff,yoff,gxoff,gyoff);
	// ...parallel to center line
	bresenham(mfa,strafYellLeft1,strafYellRight1,mRes,mRes,xoff,yoff,gxoff,gyoff);
	bresenham(mfa,strafBlueLeft1,strafBlueRight1,mRes,mRes,xoff,yoff,gxoff,gyoff);

#ifdef DRIBBLING_CHALLENGE
	bresenham(mfa,circlineBlue1,circlineBlue2,mRes,mRes,xoff,yoff,gxoff,gyoff);
	bresenham(mfa,circlineYell1,circlineYell2,mRes,mRes,xoff,yoff,gxoff,gyoff);
#endif

	// center line
	bresenham(mfa,yellowPoleTXing,bluePoleTXing,mRes,mRes,xoff,yoff,gxoff,gyoff);

	bresenham_circle(mfa,(0.f+xoff)/mRes+mGridXoff,(0.f+yoff)/mRes+mGridYoff,(float)100.0 * field->centerCircleDiameter()/mRes/2);

	// * LiuW: At RoboCup 2009, There will be only two X-Markers left.                    * //
	// *       Left the privious MiddleYellowMarker & MiddleBlueMarker and change name to * //
	// *       YellowMarker & BlueMarker.                                                 * //
	bool useMarkers = false;
	if(useMarkers){
#warning Decide whether this needs to be used!
// 		mfa((xoff+pf.worldObjectPos(YellowMarker).x*100)/mRes+mGridXoff,
// 			(yoff+pf.worldObjectPos(YellowMarker).y*100)/mRes+mGridYoff);
//
// 		mfa((xoff+pf.worldObjectPos(BlueMarker).x*100)/mRes+mGridXoff,
// 			(yoff+pf.worldObjectPos(BlueMarker).y*100)/mRes+mGridYoff);

/*
		mfa((xoff+pf.worldObjectPos(LeftYellowMarker).x*100)/mRes+mGridXoff,
			(yoff+pf.worldObjectPos(LeftYellowMarker).y*100)/mRes+mGridYoff);

		mfa((xoff+pf.worldObjectPos(RightYellowMarker).x*100)/mRes+mGridXoff,
			(yoff+pf.worldObjectPos(RightYellowMarker).y*100)/mRes+mGridYoff);

		mfa((xoff+pf.worldObjectPos(LeftBlueMarker).x*100)/mRes+mGridXoff,
			(yoff+pf.worldObjectPos(LeftBlueMarker).y*100)/mRes+mGridYoff);

		mfa((xoff+pf.worldObjectPos(RightBlueMarker).x*100)/mRes+mGridXoff,
			(yoff+pf.worldObjectPos(RightBlueMarker).y*100)/mRes+mGridYoff);

		mfa((xoff+pf.worldObjectPos(MiddleYellowMarker).x*100)/mRes+mGridXoff,
			(yoff+pf.worldObjectPos(MiddleYellowMarker).y*100)/mRes+mGridYoff);

		mfa((xoff+pf.worldObjectPos(MiddleBlueMarker).x*100)/mRes+mGridXoff,
			(yoff+pf.worldObjectPos(MiddleBlueMarker).y*100)/mRes+mGridYoff);
*/
	}
/*
	for(int y=0;y<mYdim;y++)
		for(int x=0;x<mXdim;x++){
			float& f = m_LineLikelihoodField[y*mXdim+x];
			f = max(f,mMinValue);
		}
		*/
}

inline std::pair<float,float>
DistToWhiteGrid::getLineLikelihood(float px, float py, float ox, float oy){
	Vec2f wpos(px+ox,py+oy);
	float d2 = ox*ox + oy*oy;
	float sigma = getSigma(d2);
	float minLikelihood = gaussian(3*sigma,0.0f,sigma);

	if(!isInsideMetric(wpos))
		return std::pair<float,float>(minLikelihood,minLikelihood);

	float f = getMetric(wpos);

	return std::pair<float,float>(
		gaussian(f,0.f,sigma),        // likelihood
		//(float)gsl_ran_gaussian_pdf(f,sigma),
		minLikelihood);                     // minLikelihood
}

template<class value_type>
inline Vec2i FieldGrid2D<value_type>::getCentimetricToGrid(const Vec2f& v)const{
	return getCentimetricToGrid(v.x(),v.y());
}
template<class value_type>
inline Vec2i FieldGrid2D<value_type>::getCentimetricToGrid(float wx, float wy)const{
	return Vec2i(
			(int)((wx + mXoff) / mRes+mGridXoff),
			(int)((wy + mYoff) / mRes+mGridYoff));
}
template<class value_type>
inline Vec2i FieldGrid2D<value_type>::getMetricToGrid(const Vec2f& v)const{
	return getMetricToGrid(v.x(),v.y());
}
template<class value_type>
inline Vec2i FieldGrid2D<value_type>::getMetricToGrid(float wx, float wy)const{
	return Vec2i(
			(int)((wx * 100.f + mXoff) / mRes+mGridXoff),
			(int)((wy * 100.f + mYoff) / mRes+mGridYoff));
}
template<class value_type>
inline Vec2f FieldGrid2D<value_type>::getGridToMetric(const Vec2i& v )const{
	return getGridToMetric(v.x(),v.y());
}
template<class value_type>
inline Vec2f FieldGrid2D<value_type>::getGridToMetric(int x, int y)const{
	return Vec2f(
			((x-mGridXoff)*mRes-mXoff)/100.f,
			((y-mGridYoff)*mRes-mYoff)/100.f);
}

}
