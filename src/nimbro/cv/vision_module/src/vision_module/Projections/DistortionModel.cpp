//DistotionModel.cpp
// Created on: May 12, 2015
//     Author: Hafez Farazi <farazi@ais.uni-bonn.de>
#include <vision_module/Projections/DistortionModel.hpp>

float DistortionModel::getDiagonalAngleView()
{
	vector<Point> plist(4);
	vector<Point2f> pres;
	plist[0] = Point(0, 0);
	plist[1] = Point(params.camera->width() - 1, params.camera->height() - 1);
	plist[2] = Point(0, params.camera->height() - 1);
	plist[3] = Point(params.camera->width() - 1, 0);
	undistortP_normalized_slow(plist, pres);

	double longestDagnal = max(GetDistance(pres[3]),
			max(GetDistance(pres[2]),
					max(GetDistance(pres[1]), GetDistance(pres[0]))));
	double len = 1;
	return Radian2Degree(atan2(longestDagnal, len)) * 2;
}

void DistortionModel::ModifoedOpenCVUndistortPoint(const CvMat* _src,
		CvMat* _dst, const CvMat* _cameraMatrix, const CvMat* _distCoeffs,
		const CvMat* matR, const CvMat* matP)
{
	double A[3][3], RR[3][3], k[12] =
	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, fx, fy, ifx, ify, cx, cy;
	CvMat matA = cvMat(3, 3, CV_64F, A), _Dk;
	CvMat _RR = cvMat(3, 3, CV_64F, RR);
	const CvPoint2D32f* srcf;
	const CvPoint2D64f* srcd;
	CvPoint2D32f* dstf;
	CvPoint2D64f* dstd;
	int stype, dtype;
	int sstep, dstep;
	int i, j, n, iters = 1;

	CV_Assert(
			CV_IS_MAT(_src) && CV_IS_MAT(_dst) && (_src->rows == 1 || _src->cols == 1) && (_dst->rows == 1 || _dst->cols == 1) && _src->cols + _src->rows - 1 == _dst->rows + _dst->cols - 1 && (CV_MAT_TYPE(_src->type) == CV_32FC2 || CV_MAT_TYPE(_src->type) == CV_64FC2) && (CV_MAT_TYPE(_dst->type) == CV_32FC2 || CV_MAT_TYPE(_dst->type) == CV_64FC2));

	CV_Assert(
			CV_IS_MAT(_cameraMatrix) && _cameraMatrix->rows == 3 && _cameraMatrix->cols == 3);

	cvConvert(_cameraMatrix, &matA);

	if (_distCoeffs)
	{
		CV_Assert(
				CV_IS_MAT(_distCoeffs) && (_distCoeffs->rows == 1 || _distCoeffs->cols == 1) && (_distCoeffs->rows*_distCoeffs->cols == 4 || _distCoeffs->rows*_distCoeffs->cols == 5 || _distCoeffs->rows*_distCoeffs->cols == 8 || _distCoeffs->rows*_distCoeffs->cols == 12));

		_Dk = cvMat(_distCoeffs->rows, _distCoeffs->cols,
				CV_MAKETYPE(CV_64F, CV_MAT_CN(_distCoeffs->type)), k);

		cvConvert(_distCoeffs, &_Dk);
		iters = 20;
	}

	if (matR)
	{
		CV_Assert(CV_IS_MAT(matR) && matR->rows == 3 && matR->cols == 3);
		cvConvert(matR, &_RR);
	}
	else
		cvSetIdentity(&_RR);

	if (matP)
	{
		double PP[3][3];
		CvMat _P3x3, _PP = cvMat(3, 3, CV_64F, PP);
		CV_Assert(
				CV_IS_MAT(matP) && matP->rows == 3 && (matP->cols == 3 || matP->cols == 4));
		cvConvert(cvGetCols(matP, &_P3x3, 0, 3), &_PP);
		cvMatMul(&_PP, &_RR, &_RR);
	}

	srcf = (const CvPoint2D32f*) _src->data.ptr;
	srcd = (const CvPoint2D64f*) _src->data.ptr;
	dstf = (CvPoint2D32f*) _dst->data.ptr;
	dstd = (CvPoint2D64f*) _dst->data.ptr;
	stype = CV_MAT_TYPE(_src->type);
	dtype = CV_MAT_TYPE(_dst->type);
	sstep = _src->rows == 1 ? 1 : _src->step / CV_ELEM_SIZE(stype);
	dstep = _dst->rows == 1 ? 1 : _dst->step / CV_ELEM_SIZE(dtype);

	n = _src->rows + _src->cols - 1;

	fx = A[0][0];
	fy = A[1][1];
	ifx = 1. / fx;
	ify = 1. / fy;
	cx = A[0][2];
	cy = A[1][2];

	for (i = 0; i < n; i++)
	{
		double x, y, x0, y0;
		if (stype == CV_32FC2)
		{
			x = srcf[i * sstep].x;
			y = srcf[i * sstep].y;
		}
		else
		{
			x = srcd[i * sstep].x;
			y = srcd[i * sstep].y;
		}

		x0 = x = (x - cx) * ifx;
		y0 = y = (y - cy) * ify;

		// compensate distortion iteratively
		for (j = 0; j < iters; j++)
		{
			double r2 = x * x + y * y;
			double icdist = (1 + ((k[7] * r2 + k[6]) * r2 + k[5]) * r2)
					/ (1 + ((k[4] * r2 + k[1]) * r2 + k[0]) * r2);
			double deltaX = 2 * k[2] * x * y + k[3] * (r2 + 2 * x * x)
					+ k[8] * r2 + k[9] * r2 * r2;
			double deltaY = k[2] * (r2 + 2 * y * y) + 2 * k[3] * x * y
					+ k[10] * r2 + k[11] * r2 * r2;
			x = (x0 - deltaX) * icdist;
			y = (y0 - deltaY) * icdist;
		}

		double xx = RR[0][0] * x + RR[0][1] * y + RR[0][2];
		double yy = RR[1][0] * x + RR[1][1] * y + RR[1][2];
		double ww = 1. / (RR[2][0] * x + RR[2][1] * y + RR[2][2]);
		x = xx * ww;
		y = yy * ww;

		if (dtype == CV_32FC2)
		{
			dstf[i * dstep].x = (float) x;
			dstf[i * dstep].y = (float) y;
		}
		else
		{
			dstd[i * dstep].x = x;
			dstd[i * dstep].y = y;
		}
	}
}

void DistortionModel::ModifoedOpenCVUndistortPoint(InputArray _src,
		OutputArray _dst, InputArray _cameraMatrix, InputArray _distCoeffs,
		InputArray _Rmat, InputArray _Pmat)
{
	Mat src = _src.getMat(), cameraMatrix = _cameraMatrix.getMat();
	Mat distCoeffs = _distCoeffs.getMat(), R = _Rmat.getMat(), P =
			_Pmat.getMat();

	CV_Assert(
			src.isContinuous() && (src.depth() == CV_32F || src.depth() == CV_64F) && ((src.rows == 1 && src.channels() == 2) || src.cols*src.channels() == 2));

	_dst.create(src.size(), src.type(), -1, true);
	Mat dst = _dst.getMat();

	CvMat _csrc = src, _cdst = dst, _ccameraMatrix = cameraMatrix;
	CvMat matR, matP, _cdistCoeffs, *pR = 0, *pP = 0, *pD = 0;
	if (!R.empty())
		pR = &(matR = R);
	if (!P.empty())
		pP = &(matP = P);
	if (!distCoeffs.empty())
		pD = &(_cdistCoeffs = distCoeffs);
	ModifoedOpenCVUndistortPoint(&_csrc, &_cdst, &_ccameraMatrix, pD, pR, pP);
}

bool DistortionModel::undistortP_normalized_slow(const vector<Point> contour,
		vector<Point2f> &resCountour)
{
	resCountour.resize(contour.size()); //allocate result
	cv::Mat src = cv::Mat(1, contour.size(), CV_64FC2);
	for (uint32_t i = 0; i < contour.size(); i++)
	{
		src.at<cv::Vec2d>(0, i)[0] = contour[i].x;
		src.at<cv::Vec2d>(0, i)[1] = contour[i].y;
	}

	cv::Mat norm = cv::Mat(1, contour.size(), CV_64FC2);
	ModifoedOpenCVUndistortPoint(src, norm, cameraMatrix, distCoeffs, noArray(),
			noArray());

	for (uint32_t i = 0; i < contour.size(); i++)
	{
		cv::Vec2d dv = norm.at<cv::Vec2d>(0, i);
		float x = (dv[0]);
		float y = (dv[1]);
		resCountour[i] = Point2f(x, y);
	}
	return true;
}

bool DistortionModel::distortP_normalized_slow(const vector<Point3f> contour,
		vector<Point2f> &resCountour)
{
	resCountour.resize(contour.size()); //allocate result
	cv::Mat rVec(3, 1, cv::DataType<double>::type); // Rotation vector
	rVec.at<double>(0) = 0;
	rVec.at<double>(1) = 0;
	rVec.at<double>(2) = 0;

	cv::Mat tVec(3, 1, cv::DataType<double>::type); // Translation vector
	tVec.at<double>(0) = 0;
	tVec.at<double>(1) = 0;
	tVec.at<double>(2) = 0;

	cv::projectPoints(contour, rVec, tVec, cameraMatrix, distCoeffs,
			resCountour);
	return true;
}

bool DistortionModel::DistortPFull(const vector<Point> contour,
		vector<Point> &resCountour)
{
	resCountour.resize(contour.size()); //allocate result
	vector<Point2f> resCountourFloat(contour.size());
	const int W = params.camera->width();
	const int H = params.camera->height();

	const int siX = params.camera->widthUnDistortion();
	const int siY = params.camera->heightUnDistortion();

	int offsetx = (siX - W) / 2.;
	int offsety = (siY - H) / 2.;
	double fx = cameraMatrix.at<double>(0, 0);
	double fy = cameraMatrix.at<double>(1, 1);
	double cx = cameraMatrix.at<double>(0, 2);
	double cy = cameraMatrix.at<double>(1, 2);
	vector<Point3f> contour3f(contour.size());
	for (size_t i = 0; i < contour.size(); i++)
	{
		contour3f[i] = Point3f((contour[i].x - (offsetx + cx)) / fx,
				(contour[i].y - (offsety + cy)) / fy, 1);
	}
	cv::Mat rVec(3, 1, cv::DataType<double>::type); // Rotation vector
	rVec.at<double>(0) = 0;
	rVec.at<double>(1) = 0;
	rVec.at<double>(2) = 0;
	cv::Mat tVec(3, 1, cv::DataType<double>::type); // Translation vector
	tVec.at<double>(0) = 0;
	tVec.at<double>(1) = 0;
	tVec.at<double>(2) = 0;
	cv::projectPoints(contour3f, rVec, tVec, cameraMatrix, distCoeffs,
			resCountourFloat);
	for (uint32_t i = 0; i < resCountourFloat.size(); i++)
	{
		resCountour[i] = Point((int) round(resCountourFloat[i].x),
				(int) round(resCountourFloat[i].y));
	}
	return true;
}

bool DistortionModel::DistortP(const vector<Point> contour,
		vector<Point> &resCountour)
{
	resCountour.resize(contour.size()); //allocate result
	vector<Point2f> resCountourFloat(contour.size());
	const int W = params.camera->width();
	const int H = params.camera->height();

	const int siX = params.camera->widthUnDistortion();
	const int siY = params.camera->heightUnDistortion();

	int offsetx = (siX - W) / 2.;
	int offsety = (siY - H) / 2.;

	double fx = cameraMatrix.at<double>(0, 0);
	double fy = cameraMatrix.at<double>(1, 1);
	double cx = cameraMatrix.at<double>(0, 2);
	double cy = cameraMatrix.at<double>(1, 2);

	vector<Point3f> contour3f(contour.size());
	for (size_t i = 0; i < contour.size(); i++)
	{
		contour3f[i] = Point3f((contour[i].x - (offsetx + cx)) / fx,
				(contour[i].y - (offsety + cy)) / fy, 1);
	}

	cv::Mat rVec(3, 1, cv::DataType<double>::type); // Rotation vector
	rVec.at<double>(0) = 0;
	rVec.at<double>(1) = 0;
	rVec.at<double>(2) = 0;

	cv::Mat tVec(3, 1, cv::DataType<double>::type); // Translation vector
	tVec.at<double>(0) = 0;
	tVec.at<double>(1) = 0;
	tVec.at<double>(2) = 0;

	cv::projectPoints(contour3f, rVec, tVec, cameraMatrix, distCoeffs,
			resCountourFloat);

	for (uint32_t i = 0; i < resCountourFloat.size(); i++)
	{
		resCountour[i] = Point(
				std::max(
						std::min((int) round(resCountourFloat[i].x),
								params.camera->width() - 1), 0),
				std::max(
						std::min((int) round(resCountourFloat[i].y),
								params.camera->height() - 1), 0));
	}
	return true;
}

void DistortionModel::undistortP_slow(const vector<Point> contour,
		vector<Point> &resCountour)
{
	resCountour.resize(contour.size()); //allocate result
	const int W = params.camera->width();
	const int H = params.camera->height();

	cv::Mat src = cv::Mat(1, contour.size(), CV_32FC2);
	for (uint32_t i = 0; i < contour.size(); i++)
	{
		src.at<cv::Vec2f>(0, i)[0] = std::max(std::min(contour[i].x, W), 0);

		src.at<cv::Vec2f>(0, i)[1] = std::max(std::min(contour[i].y, H), 0);

	}

	cv::Mat norm = cv::Mat(1, contour.size(), CV_32FC2);
	ModifoedOpenCVUndistortPoint(src, norm, cameraMatrix, distCoeffs, noArray(),
			cameraMatrix);

	for (uint32_t i = 0; i < contour.size(); i++)
	{
		cv::Vec2f dv = norm.at<cv::Vec2f>(0, i);
		int x = (int) (dv[0]);
		int y = (int) (dv[1]);
		resCountour[i] = Point(x, y);
	}
}

bool DistortionModel::UndistortP(const vector<Point> &contour,
		vector<Point> &resCountour)
{
	resCountour.resize(contour.size()); //allocate result

	const int W = params.camera->width();
	const int H = params.camera->height();

	for (uint32_t i = 0; i < contour.size(); i++)
	{
		int x = contour[i].x;
		int y = contour[i].y;
		if (x < 0 || y < 0 || x >= W || y >= H)
		{
			ROS_ERROR("Error In Programming (x =%d, y =%d)", x, y);
			return false;
		}

		Point tmp = distortionVector[y * W + x];
		resCountour[i] = Point(tmp.x, tmp.y);

	}
	return true;
}

bool DistortionModel::UndistortP(const vector<Point> &contour,
		vector<Point2f> &resCountour)
{
	resCountour.resize(contour.size()); //allocate result
	int W = params.camera->width();
	int H = params.camera->height();
	for (uint32_t i = 0; i < contour.size(); i++)
	{
		int x = contour[i].x;
		int y = contour[i].y;
		if (x < 0 || y < 0 || x >= W || y >= H)
		{
			ROS_ERROR("Error In Programming (x =%d, y =%d)", x, y);
			return false;
		}

		resCountour[i] = distortionVector[y * W + x];
	}
	return true;
}

bool DistortionModel::UndistortP(const Point &inPoint, Point2f &resPoint)
{
	int W = params.camera->width();
	int H = params.camera->height();
	int x = inPoint.x;
	int y = inPoint.y;
	if (x < 0 || y < 0 || x >= W || y >= H)
	{
		ROS_ERROR("Error In Programming (x =%d, y =%d)", x, y);
		return false;
	}
	resPoint = distortionVector[y * W + x];
	return true;
}

void DistortionModel::CreateUndistortFull(const Mat &rawImg, Mat &res,
		Scalar bg = blackColor())
{
	const int W = params.camera->width();
	const int H = params.camera->height();
	const int siX = params.camera->widthUnDistortion();
	const int siY = params.camera->heightUnDistortion();
	vector<Point> p(siY * siX), resP;
	int ctmp = 0;
	for (int y = 0; y < siY; y++)
	{
		for (int x = 0; x < siX; x++)
		{
			p[ctmp++] = Point(x, y);
		}
	}
	DistortPFull(p, resP);
	res = Mat::zeros(Size(siX, siY), CV_8UC3);
	res = bg;
	int counter = 0;
	const int resChannels = res.channels();
	const int resSize = siX * siY;
	uchar* raw_D = rawImg.data;
	uchar* tmp_D = res.data;
	for (int i = 0; i < resSize; i++)
	{
		int x = resP[counter].x;
		int y = resP[counter].y;
		counter++;
		tmp_D += resChannels;

		if (x < 0 || y < 0 || x >= W || y >= H)
		{
			continue;
		}
		uchar* currentP_tmp_D = raw_D + (((y * W) + x) * resChannels);
		tmp_D[0] = currentP_tmp_D[0];
		tmp_D[1] = currentP_tmp_D[1];
		tmp_D[2] = currentP_tmp_D[2];
	}
}

void DistortionModel::CreateUndistort(const Mat &rawImg, Mat &res)
{
	const int W = params.camera->width();
	const int H = params.camera->height();
	vector<Point> p(H * W), resP;
	int ctmp = 0;
	for (int y = 0; y < H; y++)
	{
		for (int x = 0; x < W; x++)
		{
			p[ctmp++] = Point(x, y);
		}
	}
	UndistortP(p, resP);

	const int siX = params.camera->widthUnDistortion();
	const int siY = params.camera->heightUnDistortion();
	res = Mat::zeros(Size(siX, siY), CV_8UC3);

	int counter = 0;

	const int rawChannels = rawImg.channels();
	const int rawSize = rawImg.rows * rawImg.cols;
	uchar* raw_D = rawImg.data;
	uchar* tmp_D = res.data;

	for (int i = 0; i < rawSize; i++)
	{
		int x = resP[counter].x;
		int y = resP[counter].y;
		counter++;
		raw_D += rawChannels;
		if (x < 0 || y < 0 || y >= siY || x >= siX)
		{
			ROS_ERROR("Error In Programming %d %d ", x, y);
			continue;
		}
		uchar* currentP_tmp_D = tmp_D + (((y * siX) + x) * rawChannels);
		currentP_tmp_D[0] = raw_D[0];
		currentP_tmp_D[1] = raw_D[1];
		currentP_tmp_D[2] = raw_D[2];
	}
}

bool DistortionModel::Init()
{
	string tmpCalibPath = "";
	try
	{
		tmpCalibPath = params.configPath + params.calib->filePath();
		ROS_INFO("cCfile path = %s", tmpCalibPath.c_str());
		cv::FileStorage fs(tmpCalibPath, FileStorage::READ);
		fs["cameraMatrix"] >> cameraMatrix;
		fs["distCoeffs"] >> distCoeffs;
	} catch (cv::Exception& e)
	{
		ROS_ERROR("Problem in cCfile @ %s", tmpCalibPath.c_str());
		ROS_ERROR("exception caught: %s", e.what());
		return false;
	}

	if(cameraMatrix.empty() || distCoeffs.empty())
	{
		ROS_ERROR("Problem in cCfile content");
		return false;
	}

	//Because the original camera matrix was created with 640*480
	{
		cameraMatrix.at<double>(Point(0, 0)) *= params.camera->width();
		cameraMatrix.at<double>(Point(0, 0)) /= 640;
		cameraMatrix.at<double>(Point(2, 0)) *= params.camera->width();
		cameraMatrix.at<double>(Point(2, 0)) /= 640;
		cameraMatrix.at<double>(Point(1, 1)) *= params.camera->height();
		cameraMatrix.at<double>(Point(1, 1)) /= 480;
		cameraMatrix.at<double>(Point(2, 1)) *= params.camera->height();
		cameraMatrix.at<double>(Point(2, 1)) /= 480;
	}
	const int W = params.camera->width();
	const int H = params.camera->height();
	distortionVector.resize(H * W);
	vector<Point> center(1), resC;
	center[0] = Point(params.camera->width() / 2, params.camera->height() / 2);
	undistortP_slow(center, resC);
	ROS_INFO("Center of Image = [%d, %d]", resC[0].x, resC[0].y);
	vector<Point> p(H * W), resP;
	int ctmp = 0;
	for (int y = 0; y < H; y++)
	{
		for (int x = 0; x < W; x++)
		{
			p[ctmp++] = Point(x, y);
		}
	}
	undistortP_slow(p, resP);

	int maxW = -999999;
	int maxH = -999999;
	for (int i = 0; i < H * W; i++)
	{
		int x = resP[i].x;
		int y = resP[i].y;

		maxW = max(abs(resC[0].x - x), maxW);
		maxH = max(abs(resC[0].y - y), maxH);

	}

	params.camera->widthUnDistortion.set((maxW * 2) + 1);
	params.camera->heightUnDistortion.set((maxH * 2) + 1);
	const int siX = params.camera->widthUnDistortion();
	const int siY = params.camera->heightUnDistortion();

	int offsetx = (siX - W) / 2.;
	int offsety = (siY - H) / 2.;
	for (int i = 0; i < H * W; i++)
	{
		int x = resP[i].x;
		int y = resP[i].y;
		int xP = p[i].x;
		int yP = p[i].y;
		distortionVector[(yP * W) + xP] = Point2f(x + offsetx, y + offsety);
	}

	ROS_INFO("Calculated Boundry if symetrical => ( width = %d , heigth = %d)",
			params.camera->widthUnDistortion(),
			params.camera->heightUnDistortion());

	return true;
}
