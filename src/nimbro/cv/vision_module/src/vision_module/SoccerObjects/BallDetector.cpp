//BallDetector.cpp
// Created on: May 12, 2015
//     Author: Hafez Farazi <farazi@ais.uni-bonn.de>

#include <vision_module/SoccerObjects/BallDetector.hpp>

#define PR(x) params.ball->x()

#define CV_IMPLEMENT_QSORT_EX( func_name, T, LT, user_data_type )                   \
void func_name( T *array, size_t total, user_data_type aux )                        \
{                                                                                   \
    int isort_thresh = 7;                                                           \
    T t;                                                                            \
    int sp = 0;                                                                     \
                                                                                    \
    struct                                                                          \
    {                                                                               \
        T *lb;                                                                      \
        T *ub;                                                                      \
    }                                                                               \
    stack[48];                                                                      \
                                                                                    \
    aux = aux;                                                                      \
                                                                                    \
    if( total <= 1 )                                                                \
        return;                                                                     \
                                                                                    \
    stack[0].lb = array;                                                            \
    stack[0].ub = array + (total - 1);                                              \
                                                                                    \
    while( sp >= 0 )                                                                \
    {                                                                               \
        T* left = stack[sp].lb;                                                     \
        T* right = stack[sp--].ub;                                                  \
                                                                                    \
        for(;;)                                                                     \
        {                                                                           \
            int i, n = (int)(right - left) + 1, m;                                  \
            T* ptr;                                                                 \
            T* ptr2;                                                                \
                                                                                    \
            if( n <= isort_thresh )                                                 \
            {                                                                       \
            insert_sort:                                                            \
                for( ptr = left + 1; ptr <= right; ptr++ )                          \
                {                                                                   \
                    for( ptr2 = ptr; ptr2 > left && LT(ptr2[0],ptr2[-1]); ptr2--)   \
                        CV_SWAP( ptr2[0], ptr2[-1], t );                            \
                }                                                                   \
                break;                                                              \
            }                                                                       \
            else                                                                    \
            {                                                                       \
                T* left0;                                                           \
                T* left1;                                                           \
                T* right0;                                                          \
                T* right1;                                                          \
                T* pivot;                                                           \
                T* a;                                                               \
                T* b;                                                               \
                T* c;                                                               \
                int swap_cnt = 0;                                                   \
                                                                                    \
                left0 = left;                                                       \
                right0 = right;                                                     \
                pivot = left + (n/2);                                               \
                                                                                    \
                if( n > 40 )                                                        \
                {                                                                   \
                    int d = n / 8;                                                  \
                    a = left, b = left + d, c = left + 2*d;                         \
                    left = LT(*a, *b) ? (LT(*b, *c) ? b : (LT(*a, *c) ? c : a))     \
                                      : (LT(*c, *b) ? b : (LT(*a, *c) ? a : c));    \
                                                                                    \
                    a = pivot - d, b = pivot, c = pivot + d;                        \
                    pivot = LT(*a, *b) ? (LT(*b, *c) ? b : (LT(*a, *c) ? c : a))    \
                                      : (LT(*c, *b) ? b : (LT(*a, *c) ? a : c));    \
                                                                                    \
                    a = right - 2*d, b = right - d, c = right;                      \
                    right = LT(*a, *b) ? (LT(*b, *c) ? b : (LT(*a, *c) ? c : a))    \
                                      : (LT(*c, *b) ? b : (LT(*a, *c) ? a : c));    \
                }                                                                   \
                                                                                    \
                a = left, b = pivot, c = right;                                     \
                pivot = LT(*a, *b) ? (LT(*b, *c) ? b : (LT(*a, *c) ? c : a))        \
                                   : (LT(*c, *b) ? b : (LT(*a, *c) ? a : c));       \
                if( pivot != left0 )                                                \
                {                                                                   \
                    CV_SWAP( *pivot, *left0, t );                                   \
                    pivot = left0;                                                  \
                }                                                                   \
                left = left1 = left0 + 1;                                           \
                right = right1 = right0;                                            \
                                                                                    \
                for(;;)                                                             \
                {                                                                   \
                    while( left <= right && !LT(*pivot, *left) )                    \
                    {                                                               \
                        if( !LT(*left, *pivot) )                                    \
                        {                                                           \
                            if( left > left1 )                                      \
                                CV_SWAP( *left1, *left, t );                        \
                            swap_cnt = 1;                                           \
                            left1++;                                                \
                        }                                                           \
                        left++;                                                     \
                    }                                                               \
                                                                                    \
                    while( left <= right && !LT(*right, *pivot) )                   \
                    {                                                               \
                        if( !LT(*pivot, *right) )                                   \
                        {                                                           \
                            if( right < right1 )                                    \
                                CV_SWAP( *right1, *right, t );                      \
                            swap_cnt = 1;                                           \
                            right1--;                                               \
                        }                                                           \
                        right--;                                                    \
                    }                                                               \
                                                                                    \
                    if( left > right )                                              \
                        break;                                                      \
                    CV_SWAP( *left, *right, t );                                    \
                    swap_cnt = 1;                                                   \
                    left++;                                                         \
                    right--;                                                        \
                }                                                                   \
                                                                                    \
                if( swap_cnt == 0 )                                                 \
                {                                                                   \
                    left = left0, right = right0;                                   \
                    goto insert_sort;                                               \
                }                                                                   \
                                                                                    \
                n = MIN( (int)(left1 - left0), (int)(left - left1) );               \
                for( i = 0; i < n; i++ )                                            \
                    CV_SWAP( left0[i], left[i-n], t );                              \
                                                                                    \
                n = MIN( (int)(right0 - right1), (int)(right1 - right) );           \
                for( i = 0; i < n; i++ )                                            \
                    CV_SWAP( left[i], right0[i-n+1], t );                           \
                n = (int)(left - left1);                                            \
                m = (int)(right1 - right);                                          \
                if( n > 1 )                                                         \
                {                                                                   \
                    if( m > 1 )                                                     \
                    {                                                               \
                        if( n > m )                                                 \
                        {                                                           \
                            stack[++sp].lb = left0;                                 \
                            stack[sp].ub = left0 + n - 1;                           \
                            left = right0 - m + 1, right = right0;                  \
                        }                                                           \
                        else                                                        \
                        {                                                           \
                            stack[++sp].lb = right0 - m + 1;                        \
                            stack[sp].ub = right0;                                  \
                            left = left0, right = left0 + n - 1;                    \
                        }                                                           \
                    }                                                               \
                    else                                                            \
                        left = left0, right = left0 + n - 1;                        \
                }                                                                   \
                else if( m > 1 )                                                    \
                    left = right0 - m + 1, right = right0;                          \
                else                                                                \
                    break;                                                          \
            }                                                                       \
        }                                                                           \
    }                                                                               \
}

#define CV_IMPLEMENT_QSORT( func_name, T, cmp )  \
    CV_IMPLEMENT_QSORT_EX( func_name, T, cmp, int )
#define hough_cmp_gt(l1,l2) (aux[l1] > aux[l2])

static CV_IMPLEMENT_QSORT_EX( icvHoughSortDescent32s, int, hough_cmp_gt, const int* )

static void myicvHoughCirclesGradient(CvMat* img, float dp, float min_dist,
		int min_radius, int max_radius, int canny_threshold, int acc_threshold,
		CvSeq* circles, int circles_max)
{

	const int SHIFT = 10, ONE = 1 << SHIFT;
	cv::Ptr<CvMat> dx, dy;
	cv::Ptr<CvMat> edges, accum, dist_buf;
	std::vector<int> sort_buf;
	cv::Ptr<CvMemStorage> storage;

	int x, y, i, j, k, center_count, nz_count;
	float min_radius2 = (float) min_radius * min_radius;
	float max_radius2 = (float) max_radius * max_radius;
	int rows, cols, arows, acols;
	int astep, *adata;
	float* ddata;
	CvSeq *nz, *centers;
	float idp, dr;
	CvSeqReader reader;

	{
		edges = cvCreateMat(img->rows, img->cols, CV_8UC1);
		cvCanny(img, edges, MAX(canny_threshold / 2, 1), canny_threshold, 3);

	}
	{
		dx = cvCreateMat(img->rows, img->cols, CV_16SC1);
		dy = cvCreateMat(img->rows, img->cols, CV_16SC1);
		cvSobel(img, dx, 1, 0, 3);
		cvSobel(img, dy, 0, 1, 3);

	}
	if (dp < 1.f)
		dp = 1.f;
	idp = 1.f / dp;
	accum = cvCreateMat(cvCeil(img->rows * idp) + 2,
			cvCeil(img->cols * idp) + 2, CV_32SC1);
	cvZero(accum);

	storage = cvCreateMemStorage();
	nz = cvCreateSeq( CV_32SC2, sizeof(CvSeq), sizeof(CvPoint), storage);
	centers = cvCreateSeq( CV_32SC1, sizeof(CvSeq), sizeof(int), storage);

	rows = img->rows;
	cols = img->cols;
	arows = accum->rows - 2;
	acols = accum->cols - 2;
	adata = accum->data.i;
	astep = accum->step / sizeof(adata[0]);

	{
		for (y = 0; y < rows; y++)
		{
			const uchar* edges_row = edges->data.ptr + y * edges->step;
			const short* dx_row = (const short*) (dx->data.ptr + y * dx->step);
			const short* dy_row = (const short*) (dy->data.ptr + y * dy->step);

			for (x = 0; x < cols; x++)
			{
				float vx, vy;
				int sx, sy, x0, y0, x1, y1, r;
				CvPoint pt;

				vx = dx_row[x];
				vy = dy_row[x];

				if (!edges_row[x] || (vx == 0 && vy == 0))
					continue;

				float mag = sqrt(vx * vx + vy * vy);
				assert(mag >= 1);
				sx = cvRound((vx * idp) * ONE / mag);
				sy = cvRound((vy * idp) * ONE / mag);

				x0 = cvRound((x * idp) * ONE);
				y0 = cvRound((y * idp) * ONE);
				// Step from min_radius to max_radius in both directions of the gradient
				for (int k1 = 0; k1 < 2; k1++)
				{
					x1 = x0 + min_radius * sx;
					y1 = y0 + min_radius * sy;

					for (r = min_radius; r <= max_radius;
							x1 += sx, y1 += sy, r++)
					{
						int x2 = x1 >> SHIFT, y2 = y1 >> SHIFT;
						if ((unsigned) x2 >= (unsigned) acols
								|| (unsigned) y2 >= (unsigned) arows)
							break;
						adata[y2 * astep + x2]++;
					}

					sx = -sx;
					sy = -sy;
				}

				pt.x = x;
				pt.y = y;
				cvSeqPush(nz, &pt);
			}
		}
	}

	nz_count = nz->total;
	if (!nz_count)
		return;
	//Find possible circle centers
	for (y = 1; y < arows - 1; y++)
	{
		for (x = 1; x < acols - 1; x++)
		{
			int base = y * (acols + 2) + x;
			if (adata[base] > acc_threshold && adata[base] > adata[base - 1]
					&& adata[base] > adata[base + 1]
					&& adata[base] > adata[base - acols - 2]
					&& adata[base] > adata[base + acols + 2])
				cvSeqPush(centers, &base);
		}
	}

	center_count = centers->total;
	if (!center_count)
		return;

	sort_buf.resize(MAX(center_count, nz_count));
	cvCvtSeqToArray(centers, &sort_buf[0]);

	/// @cond
	icvHoughSortDescent32s(&sort_buf[0], center_count, adata);
	cvClearSeq(centers);
	cvSeqPushMulti(centers, &sort_buf[0], center_count);
	/// @endcond

	dist_buf = cvCreateMat(1, nz_count, CV_32FC1);
	ddata = dist_buf->data.fl;

	dr = dp;
	min_dist = MAX(min_dist, dp);
	min_dist *= min_dist;

	{

		for (i = 0; i < centers->total; i++)
		{
			int ofs = *(int*) cvGetSeqElem(centers, i);
			y = ofs / (acols + 2);
			x = ofs - (y) * (acols + 2);
			//Calculate circle's center in pixels
			float cx = (float) ((x + 0.5f) * dp),
					cy = (float) ((y + 0.5f) * dp);
			float start_dist, dist_sum;
			float r_best = 0;
			int max_count = 0;
			// Check distance with previously detected circles
			for (j = 0; j < circles->total; j++)
			{
				float* c = (float*) cvGetSeqElem(circles, j);
				if ((c[0] - cx) * (c[0] - cx) + (c[1] - cy) * (c[1] - cy)
						< min_dist)
					break;
			}

			if (j < circles->total)
				continue;
			// Estimate best radius
			cvStartReadSeq(nz, &reader);
			for (j = k = 0; j < nz_count; j++)
			{
				CvPoint pt;
				float _dx, _dy, _r2;
				CV_READ_SEQ_ELEM(pt, reader);
				_dx = cx - pt.x;
				_dy = cy - pt.y;
				_r2 = _dx * _dx + _dy * _dy;
				if (min_radius2 <= _r2 && _r2 <= max_radius2)
				{
					ddata[k] = _r2;
					sort_buf[k] = k;
					k++;
				}
			}

			int nz_count1 = k, start_idx = nz_count1 - 1;
			if (nz_count1 == 0)
				continue;
			dist_buf->cols = nz_count1;
			cvPow(dist_buf, dist_buf, 0.5);
			icvHoughSortDescent32s(&sort_buf[0], nz_count1, (int*) ddata);

			dist_sum = start_dist = ddata[sort_buf[nz_count1 - 1]];
			for (j = nz_count1 - 2; j >= 0; j--)
			{
				float d = ddata[sort_buf[j]];

				if (d > max_radius)
					break;

				if (d - start_dist > dr)
				{
					float r_cur = ddata[sort_buf[(j + start_idx) / 2]];
					if ((start_idx - j) * r_best >= max_count * r_cur
							|| (r_best < FLT_EPSILON
									&& start_idx - j >= max_count))
					{
						r_best = r_cur;
						max_count = start_idx - j;
					}
					start_dist = d;
					start_idx = j;
					dist_sum = 0;
				}
				dist_sum += d;
			}
			// Check if the circle has enough support
			if (max_count > acc_threshold)
			{
				float c[3];
				c[0] = cx;
				c[1] = cy;
				c[2] = (float) r_best;
				cvSeqPush(circles, c);
				if (circles->total > circles_max)
					return;
			}
		}
	}
}

CvSeq*
mycvHoughCircles(CvArr* src_image, void* circle_storage, int method, double dp,
		double min_dist, double param1, double param2, int min_radius,
		int max_radius)
{
	CvSeq* result = 0;

	CvMat stub, *img = (CvMat*) src_image;
	CvMat* mat = 0;
	CvSeq* circles = 0;
	CvSeq circles_header;
	CvSeqBlock circles_block;
	int circles_max = INT_MAX;
	int canny_threshold = cvRound(param1);
	int acc_threshold = cvRound(param2);

	img = cvGetMat(img, &stub);

	if (!CV_IS_MASK_ARR(img))
		CV_Error(CV_StsBadArg,
				"The source image must be 8-bit, single-channel");

	if (!circle_storage)
		CV_Error(CV_StsNullPtr, "NULL destination");

	if (dp <= 0 || min_dist <= 0 || canny_threshold <= 0 || acc_threshold <= 0)
		CV_Error(CV_StsOutOfRange,
				"dp, min_dist, canny_threshold and acc_threshold must be all positive numbers");

	min_radius = MAX(min_radius, 0);
	if (max_radius <= 0)
		max_radius = MAX(img->rows, img->cols);
	else if (max_radius <= min_radius)
		max_radius = min_radius + 2;

	if (CV_IS_STORAGE(circle_storage))
	{
		circles = cvCreateSeq( CV_32FC3, sizeof(CvSeq), sizeof(float) * 3,
				(CvMemStorage*) circle_storage);
	}
	else if (CV_IS_MAT(circle_storage))
	{
		mat = (CvMat*) circle_storage;

		if (!CV_IS_MAT_CONT(mat->type) || (mat->rows != 1 && mat->cols != 1) ||
		CV_MAT_TYPE(mat->type) != CV_32FC3)
			CV_Error(CV_StsBadArg,
					"The destination matrix should be continuous and have a single row or a single column");

		circles = cvMakeSeqHeaderForArray( CV_32FC3, sizeof(CvSeq),
				sizeof(float) * 3, mat->data.ptr, mat->rows + mat->cols - 1,
				&circles_header, &circles_block);
		circles_max = circles->total;
		cvClearSeq(circles);
	}
	else
		CV_Error(CV_StsBadArg, "Destination is not CvMemStorage* nor CvMat*");

	switch (method)
	{
	case CV_HOUGH_GRADIENT:
		myicvHoughCirclesGradient(img, (float) dp, (float) min_dist, min_radius,
				max_radius, canny_threshold, acc_threshold, circles,
				circles_max);
		break;
	default:
		CV_Error(CV_StsBadArg, "Unrecognized method id");
	}

	if (mat)
	{
		if (mat->cols > mat->rows)
			mat->cols = circles->total;
		else
			mat->rows = circles->total;
	}
	else
		result = circles;

	return result;
}

static void seqToMat(const CvSeq* seq, OutputArray _arr)
{
	if (seq && seq->total > 0)
	{
		_arr.create(1, seq->total, seq->flags, -1, true);
		Mat arr = _arr.getMat();
		cvCvtSeqToArray(seq, arr.data);
	}
	else
		_arr.release();
}

void MyHoughCircles(InputArray _image, OutputArray _circles, int method,
		double dp, double min_dist, double param1, double param2, int minRadius,
		int maxRadius)
{
	const int STORAGE_SIZE = 1 << 12;
	Ptr<CvMemStorage> storage = cvCreateMemStorage(STORAGE_SIZE);
	Mat image = _image.getMat();
	CvMat c_image = image;
	CvSeq* seq = mycvHoughCircles(&c_image, storage, method, dp, min_dist,
			param1, param2, minRadius, maxRadius);

	seqToMat(seq, _circles);
}

std::vector<BallCircleC> BallDetector::GetBall(const Mat &rawHSV,
		const vector<Point> &hullField, const Mat &fieldBinaryRaw, Mat &rawGray,
		const Mat &ballMask, const Mat &fieldConvectHull,
		CameraProjections &projection, Mat &cannyImg, GuiManager *_guiManager,
		Mat &guiImg, bool SHOWGUI)
{

	struct debugCircle
	{
		string note;
		int radius;
		Point center;
		Scalar color;
		bool isBall;
		bool isCascade;
	};

	vector<debugCircle> debugCircles;

	Mat fieldBinaryRawBlur, fieldNot;
	int distanceToCascade = params.ball->dist2cascade();
	bitwise_not(fieldBinaryRaw, fieldNot, fieldConvectHull);
	int oddBlur = params.ball->kernelBlur() * 2 + 1;
	cv::GaussianBlur(fieldNot, fieldBinaryRawBlur, cv::Size(oddBlur, oddBlur),
			params.ball->sigmaBlur(), params.ball->sigmaBlur());
	if (SHOWGUI && params.ball->showInput())
	{
		std::vector<cv::Mat> images(3);
		images.at(0) = fieldBinaryRawBlur;
		images.at(1) = fieldBinaryRawBlur;
		images.at(2) = fieldBinaryRawBlur;
		cv::merge(images, guiImg);
		circle(guiImg,
				cv::Point2d((params.camera->width() / 2),
						(params.camera->height())), distanceToCascade,
				yellowColor(), 1, 8, 0);
	}

	vector<vector<cv::Point> > contours;
	findContours(ballMask.clone(),
			contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	int HOUGHMAXDIST = params.ball->houghMaxDist();
	int HOUGHMINR = params.ball->houghMinR();
	int HOUGHMAXR = params.ball->houghMaxR();
	if (params.ball->pyrDown())
	{
		pyrDown(fieldBinaryRawBlur, fieldBinaryRawBlur);
		HOUGHMAXDIST /= 2.;
		HOUGHMINR /= 2.;
		HOUGHMAXR /= 2.;
	}

	struct enableRect
	{
		cv::Rect rec;
		bool enable;
	};

	vector<enableRect> candidateBoxs(contours.size());
	for (size_t i = 0; i < contours.size(); i++)
	{
		cv::Rect bigRec = cv::boundingRect(contours[i]);
		if (params.ball->pyrDown())
		{
			bigRec.x /= 2.;
			bigRec.y /= 2.;
			bigRec.width /= 2.;
			bigRec.height /= 2.;
		}
		if (SHOWGUI && params.ball->showBoxs())
		{
			Rect tmpShow = bigRec;
			if (params.ball->pyrDown())
			{
				tmpShow.x *= 2.;
				tmpShow.y *= 2.;
				tmpShow.width *= 2.;
				tmpShow.height *= 2.;
			}
			rectangle(guiImg, tmpShow, darkOrangeColor(), 1, 8, 0);
		}
		bigRec = asymetricScaleRec(params.ball->houghCandWRatio(),
				params.ball->houghCandWRatio(),
				params.ball->houghCandHTopRatio(),
				params.ball->houghCandHDownRatio(), bigRec,
				(params.ball->pyrDown() ? IMGPYRDOWNBOX:IMGBOX) );

		minDimentionRect(HOUGHMINR, bigRec);

		bigRec = clipRect(bigRec, (params.ball->pyrDown() ? IMGPYRDOWNBOX:IMGBOX));

		candidateBoxs[i].rec = bigRec;
		candidateBoxs[i].enable = true;

		if (SHOWGUI && params.ball->showBoxs())
		{
			Rect tmpShow = bigRec;
			if (params.ball->pyrDown())
			{
				tmpShow.x *= 2.;
				tmpShow.y *= 2.;
				tmpShow.width *= 2.;
				tmpShow.height *= 2.;
			}
			rectangle(guiImg, tmpShow, blackColor(), 1, 8, 0);
		}
	}

	int candSize = candidateBoxs.size();
	int lastCandSize = candidateBoxs.size();
	do
	{
		lastCandSize = candSize;
		for (size_t i = 0; i < candidateBoxs.size(); i++)
		{
			for (size_t j = i + 1; j < candidateBoxs.size(); j++)
			{
				if (candidateBoxs[i].enable && candidateBoxs[j].enable)
				{
					if (mergeRect(candidateBoxs[i].rec, candidateBoxs[j].rec))
					{
						candidateBoxs[j].enable = false;
						candSize--;
					}
				}
			}
		}
	} while (candSize < lastCandSize);

	Vector<CircleC> circles;
	circles.reserve(50);
	for (size_t i = 0; i < candidateBoxs.size(); i++)
	{
		if (!candidateBoxs[i].enable)
		{
			continue;
		}
		if (SHOWGUI && params.ball->showBoxs())
		{
			Rect tmpShow = candidateBoxs[i].rec;
			if (params.ball->pyrDown())
			{
				tmpShow.x *= 2.;
				tmpShow.y *= 2.;
				tmpShow.width *= 2.;
				tmpShow.height *= 2.;
			}
			rectangle(guiImg, tmpShow, yellowColor(), 2, 8, 0);
		}
		Mat portionImg = fieldBinaryRawBlur(candidateBoxs[i].rec);
		if (params.ball->pyrDown())
		{
			vector<cv::Vec3f> tmpCircles;
			//TODO: check passing canny my self (maybe by subtraction of 1 erode and current img)
			MyHoughCircles(portionImg, tmpCircles, CV_HOUGH_GRADIENT,
					params.ball->houghDP(), HOUGHMAXDIST,
					params.ball->houghCanny(), params.ball->houghCircle(),
					HOUGHMINR, HOUGHMAXR);
			for (size_t rIdx = 0; rIdx < tmpCircles.size(); rIdx++)
			{
				CircleC cir(
						Point2f(cvRound(tmpCircles[rIdx][0]),
								cvRound(tmpCircles[rIdx][1])),
						cvRound(tmpCircles[rIdx][2]) * 2.);

				cir.Center.x += candidateBoxs[i].rec.x;
				cir.Center.y += candidateBoxs[i].rec.y;
				cir.Center.x *= 2.;
				cir.Center.y *= 2.;
				circles.push_back(cir);
			}
		}
		else
		{
			vector<cv::Vec3f> tmpCircles;
			MyHoughCircles(portionImg, tmpCircles, CV_HOUGH_GRADIENT,
					params.ball->houghDP(), HOUGHMAXDIST,
					params.ball->houghCanny(), params.ball->houghCircle(),
					HOUGHMINR, HOUGHMAXR);
			for (size_t rIdx = 0; rIdx < tmpCircles.size(); rIdx++)
			{
				CircleC cir(
						Point2f(cvRound(tmpCircles[rIdx][0]),
								cvRound(tmpCircles[rIdx][1])),
						cvRound(tmpCircles[rIdx][2]));
				cir.Center.x += candidateBoxs[i].rec.x;
				cir.Center.y += candidateBoxs[i].rec.y;
				circles.push_back(cir);
			}
		}
	}

	double ratioHT = params.ball->BiggerRectHT();
	double ratioHB = params.ball->BiggerRectHB();
	double ratioW = params.ball->BiggerRectW();
	vector<BallCircleC> resBall;
	resBall.reserve(circles.size());

	debugCircles.resize(circles.size());

	vector<CircleC> CandidateBall;
	CandidateBall.reserve(circles.size());
	const int MIN_REC_AREA = HOUGHMINR * HOUGHMINR * 4;

	for (size_t i = 0; i < circles.size(); i++)
	{
		Point center = circles[i].Center;
		int radius = circles[i].Radius;

		debugCircles[i].note = "";

		int left = max(center.x - radius, 0);
		int top = max(center.y - radius, 0);
		int right = min(left + radius * 2, IMGWIDTH);
		int down = min(top + radius * 2, IMGHEIGHT);
		cv::Rect recCircle(left, top, right - left, down - top);

		Point2f realPos;
		if (!checkDistance_Rec(recCircle, projection, realPos))
		{
			if (SHOWGUI && params.ball->showAllCircle())
			{
				debugCircles[i].color = redColor();
				debugCircles[i].isBall = false;
				debugCircles[i].isCascade = false;
				debugCircles[i].center = center;
				debugCircles[i].radius = radius;
				debugCircles[i].note += "Size=BAD ";

			}
			continue;
		}
		else
		{
			debugCircles[i].note += "Size=OK ";
		}
		bool checkMinWhite = true;
		bool checkMinNotGreen = true;
		bool checkBtnGreen = true;
		int area = radius * 4 * radius;
		bool areaCheck = (area > MIN_REC_AREA);

		if (areaCheck)
		{

			debugCircles[i].note += "Area=OK ";
			if (PR(btnGCheckPercent) > 0)
			{
				Rect tmpRect = asymetricScaleRec(params.ball->btnGCheckBoxW(),
						params.ball->btnGCheckBoxW(), PR(btnGCheckBoxtop),
						PR(btnGCheckBoxdown), recCircle, IMGBOX);
				Mat recImg = fieldNot(tmpRect);
				int tmpTotalArea=tmpRect.width*tmpRect.height;
				int totalExpectedArea=(tmpTotalArea)-(M_PI*radius*radius/2.);
				int actualtotalArea= tmpTotalArea-countNonZero(recImg);
				int btnGPercent = (actualtotalArea / (double) totalExpectedArea) * 100.;
				checkBtnGreen = (btnGPercent >= PR(btnGCheckPercent));
				debugCircles[i].note+="BtnG="+std::to_string(btnGPercent)+"("+(checkBtnGreen?"OK":"BAD")+") ";
			}

			if (checkBtnGreen && params.ball->whitePercent() > 0)
			{
				Mat recImg = ballMask(recCircle);
				int countNonZ = countNonZero(recImg);
				int wPercent = (countNonZ / (double) area) * 100;
				checkMinWhite = (wPercent >= params.ball->whitePercent());
				debugCircles[i].note += "MinWhite=" + std::to_string(wPercent)
						+ "(" + (checkMinWhite ? "OK" : "BAD") + ") ";
			}

			if (checkBtnGreen && checkMinNotGreen
					&& params.ball->notGreenPercent() > 0)
			{
				Mat recImg = fieldNot(recCircle);
				int countNonZ = countNonZero(recImg);
				int area = radius * 4 * radius;
				int ngPercent = (countNonZ / (double) area) * 100;
				checkMinNotGreen =
						(ngPercent >= params.ball->notGreenPercent());
				debugCircles[i].note += "NotGreen=" + std::to_string(ngPercent)
						+ "(" + (checkMinNotGreen ? "OK" : "BAD") + ") ";
			}
		}
		else
		{
			debugCircles[i].note += "Area=BAD ";
		}

		if (areaCheck && checkMinWhite && checkMinNotGreen && checkBtnGreen)
		{

			double histRange[3];
			histRange[0] = params.ball->histogramH();
			histRange[1] = params.ball->histogramS();
			histRange[2] = params.ball->histogramV();

			CircleC circleToAdd(center, radius);
			double histResult[3];
			if (checkHistogram(rawHSV, fieldBinaryRaw, center, radius,
					histRange, histResult))
			{
				debugCircles[i].note += "Hist=OK [H="
						+ std::to_string(histResult[0]) + " S="
						+ std::to_string(histResult[1]) + " V="
						+ std::to_string(histResult[2]) + "] ";
				if (SHOWGUI
						&& (params.ball->showCandidate()
								|| params.ball->showAllCircle()))
				{
					debugCircles[i].color = greenColor();
					debugCircles[i].isBall = false;
					debugCircles[i].isCascade = false;
					debugCircles[i].center = center;
					debugCircles[i].radius = radius;
				}
				double distanceToZero = GetDistance(
						cv::Point2d((params.camera->width() / 2),
								(params.camera->height())), center);

				if (distanceToZero < distanceToCascade)
				{
					if (SHOWGUI && params.ball->showResult())
					{
						debugCircles[i].color = pinkColor();
						debugCircles[i].isBall = true;
						debugCircles[i].isCascade = false;
						debugCircles[i].center = center;
						debugCircles[i].radius = radius;
					}
					if (params.ball->subResultFromCanny())
					{
						circle(cannyImg, center, radius, blackGary(), -1);
					}
					debugCircles[i].note += " Near=OK";

					resBall.push_back(
							BallCircleC(center, radius, realPos, false));
				}
				else
				{
					debugCircles[i].note += " Near=BAD";
					CandidateBall.push_back(circleToAdd);
				}

			}
			else
			{
				debugCircles[i].note += "Hist=BAD [H="
						+ std::to_string(histResult[0]) + " S="
						+ std::to_string(histResult[1]) + " V="
						+ std::to_string(histResult[2]) + "] ";
				CandidateBall.push_back(circleToAdd);
				if (SHOWGUI
						&& (params.ball->showCandidate()
								|| params.ball->showAllCircle()))
				{
					debugCircles[i].color = blueColor();
					debugCircles[i].isBall = false;
					debugCircles[i].isCascade = false;
					debugCircles[i].center = center;
					debugCircles[i].radius = radius;
				}
			}
		}
		else
		{
			if (SHOWGUI && params.ball->showAllCircle())
			{
				debugCircles[i].color = yellowColor();
				debugCircles[i].isBall = false;
				debugCircles[i].isCascade = false;
				debugCircles[i].center = center;
				debugCircles[i].radius = radius;
			}
		}
		continue;
	}

	if (params.ball->enableHog() && resBall.size() < 1)
	{
		for (size_t iCan = 0; iCan < CandidateBall.size(); iCan++)
		{

			std::vector<cv::Rect> objects;
			cv::Rect rec = asymetricScaleRec(ratioW, ratioW, ratioHT, ratioHB,
					CircleCToRect(CandidateBall[iCan]), IMGBOX);

			cv::Mat imgGray = rawGray(rec);
			int minSizeDim = max(3,
					(int) (max(rec.size().height, rec.size().width) / 14.));
//			int minNeighborsDim = max(5,
//					(int) (min(rec.size().height, rec.size().width) / 17.));

			object_cascade.detectMultiScale(imgGray, objects,
					params.ball->cascadeScale(),
					params.ball->cascadeNinNeighbors(), 0,
					cv::Size(minSizeDim, minSizeDim), rec.size());

			for (size_t i = 0; i < objects.size(); i++)
			{
				objects[i].x += rec.x;
				objects[i].y += rec.y;
			}
			std::sort(objects.begin(), objects.end(), sorter(&projection));

			for (size_t iObj = 0; iObj < objects.size(); iObj++)
			{

				cv::Rect resRect(objects[iObj].x, objects[iObj].y,
						objects[iObj].width, objects[iObj].height);
				cv::Point2d center = GetCenter(resRect);
				if (pointPolygonTest(hullField, center, false) < 0)
				{
					continue;
				}
				Point2f realPos;
				if (!checkDistance_Rec(resRect, projection, realPos))
				{
					continue;
				}

				double histRange[3];
				histRange[0] = params.ball->histogramH_C();
				histRange[1] = params.ball->histogramS_C();
				histRange[2] = params.ball->histogramV_C();
				double histResult[3];
				if (!checkHistogram(rawHSV, fieldBinaryRaw, center,
						objects[iObj].width, histRange, histResult))
					continue;

				if (cv::countNonZero(ballMask(resRect))
						< (resRect.width * resRect.height) * 0.2)
					continue;

				if (GetDistance(center, CandidateBall[iCan].Center)
						> resRect.width * 0.5)
					continue;

				double radius = min(resRect.width / 2., resRect.height / 2.);

				if (SHOWGUI && params.ball->showResult())
				{
					debugCircle tmpDCir;
					tmpDCir.color = pinkMeloColor();
					tmpDCir.isBall = true;
					tmpDCir.isCascade = true;
					tmpDCir.center = center;
					tmpDCir.radius = radius;
					debugCircles.push_back(tmpDCir);
				}

				if (params.ball->subResultFromCanny())
				{
					circle(cannyImg, center, radius, blackGary(), -1);
				}
				resBall.push_back(BallCircleC(center, radius, realPos, true));
				break;
			}
		}
	}

	if (params.gui->debugBallCandidates())
	{
		double minDist = MAXIMGDIST + 1;
		int inx = -1;
		for (size_t i = 0; i < debugCircles.size(); i++)
		{
			double dist = GetDistance(params.ball->debugThisPos,
					debugCircles[i].center);
			if (dist < minDist)
			{
				minDist = dist;
				inx = i;
			}
			circle(guiImg, debugCircles[i].center, debugCircles[i].radius,
					debugCircles[i].color, 1, 8, 0);
		}
		if (inx >= 0)
		{
			circle(guiImg, debugCircles[inx].center, debugCircles[inx].radius,
					debugCircles[inx].color, 3, 8, 0);
			if (params.ball->debugThis)
			{
				_guiManager->writeGuiConsoleFormat(whiteColor(),
						debugCircles[inx].note.c_str());
				params.ball->debugThis = false;
			}
		}
	}
	else
	{
		for (size_t i = 0; i < debugCircles.size(); i++)
		{
			if (!debugCircles[i].isBall)
			{
				circle(guiImg, debugCircles[i].center, debugCircles[i].radius,
						debugCircles[i].color, 2, 8, 0);
			}
		}
		if (params.ball->mergeResult())
		{
			int resSize = resBall.size();
			int lastResSize = resBall.size();
			do
			{
				lastResSize = resSize;
				for (size_t i = 0; i < resBall.size(); i++)
				{
					for (size_t j = i + 1; j < resBall.size(); j++)
					{
						if (resBall[i].enable && resBall[j].enable)
						{
							if (hasIntersection(resBall[i], resBall[j]))
							{
								resBall[j].enable = false;
								resSize--;

								CircleC tmpC = mergeCircleC(resBall[i],
										resBall[j]);
								Point2f realPos;
								if (checkDistance_Rec(CircleCToRect(tmpC),
										projection, realPos))
								{
									resBall[i].RealPos = realPos;
									resBall[i].Center = tmpC.Center;
									resBall[i].Radius = tmpC.Radius;
									resBall[i].isMerged = true;
								}
							}
						}
					}
				}
			} while (resSize < lastResSize);

			if (SHOWGUI && params.ball->showResult())
			{
				for (size_t i = 0; i < resBall.size(); i++)
				{
					if (!resBall[i].enable)
					{
						circle(guiImg, resBall[i].Center, resBall[i].Radius,
								Scalar(240, 206, 233), 1, 8, 0);
					}
				}
				for (size_t i = 0; i < resBall.size(); i++)
				{
					if (resBall[i].enable)
					{
						circle(guiImg, resBall[i].Center, resBall[i].Radius,
								resBall[i].isCascade ?
										pinkMeloColor() : pinkColor(), 2, 8, 0);
					}
				}
			}
		}
		else
		{
			for (size_t i = 0; i < debugCircles.size(); i++)
			{
				if (debugCircles[i].isBall)
				{
					circle(guiImg, debugCircles[i].center,
							debugCircles[i].radius, debugCircles[i].color, 2, 8,
							0);
				}
			}
		}
	}
	boost::remove_erase_if(resBall, [](const BallCircleC& _in)
	{
		return !_in.enable;
	});

	std::sort(resBall.begin(), resBall.end(),
			[](const BallCircleC& a, const BallCircleC& b)
			{
				return GetDistance(a.RealPos) < GetDistance(b.RealPos);
			});

	return resBall;
}

bool BallDetector::checkHistogram(const Mat &rawHSV, const Mat &fieldBinaryRaw,
		const vector<cv::Point> &con, double minHistogramDiff[3],
		double histResult[3])
{
	vector<cv::Point> conRoi;

	cv::Rect boundingRectangle = boundingRect(con);
	cv::Mat hsvRoi = rawHSV(boundingRectangle);
	cv::Mat notFieldMaskRoi = 255 - fieldBinaryRaw(boundingRectangle);
	cv::Mat paintedContour = cv::Mat::zeros(hsvRoi.size(), CV_8UC1);
	for (size_t i = 0; i < con.size(); i++)
	{
		conRoi.push_back(
				cv::Point(con[i].x - boundingRectangle.x,
						con[i].y - boundingRectangle.y));
	}
	vector<vector<cv::Point> > conRoiS = vector<vector<cv::Point> >(1, conRoi); //Nasty stuff just because of no implementation on draw one contour in opencv!
	drawContours(paintedContour, conRoiS, -1, grayWhite(),
	CV_FILLED, 8); // Draw the convexhull of the field

	cv::Mat justBallMask;
	paintedContour.copyTo(justBallMask, notFieldMaskRoi);

	cv::Mat forGuiShowOnlyBall;
	hsvRoi.copyTo(forGuiShowOnlyBall, justBallMask);
	//cv::imshow("hsvRoi", justBallMask);
	//cv::waitKey(100);
	//cv::imshow("forGuiShowOnlyBall", forGuiShowOnlyBall);
	//cv::waitKey(100);

	return checkHistogramInPic(hsvRoi, justBallMask, minHistogramDiff,
			histResult);

}

bool BallDetector::checkHistogram(const Mat &rawHSV, const Mat &fieldBinaryRaw,
		cv::Point center, int radius, double minHistogramDiff[3],
		double histResult[3])
{
	cv::Rect recCircle(center.x - radius, center.y - radius, radius * 2,
			radius * 2);
	if (center.x - radius < 0 || center.y - radius < 0
			|| center.x + radius * 2 >= params.camera->width()
			|| center.y + radius * 2 >= params.camera->height())
	{
		//printf("Because the calculated radius is bigger than image \n");
		recCircle.x = max(0, center.x - radius);
		recCircle.y = max(0, center.y - radius);
		if (recCircle.x + radius * 2 > params.camera->width() - 1)
		{
			recCircle.width = params.camera->width() - 1 - recCircle.x;
		}
		else
		{
			recCircle.width = radius * 2;
		}
		if (recCircle.y + radius * 2 > params.camera->height() - 1)
		{
			recCircle.height = params.camera->height() - 1 - recCircle.y;
		}
		else
		{
			recCircle.height = radius * 2;
		}
		//  return false;
	}

	cv::Mat hsvRoi = rawHSV(recCircle);
	cv::Mat notFieldMaskRoi = 255 - fieldBinaryRaw(recCircle);
	cv::Mat paintedContour = cv::Mat::zeros(hsvRoi.size(), CV_8UC1);

	cv::circle(paintedContour,
			cv::Point(center.x - recCircle.x, center.y - recCircle.y), radius,
			grayWhite(), -1);

	cv::Mat justBallMask;
	paintedContour.copyTo(justBallMask, notFieldMaskRoi);

	//cv::Mat forGuiShowOnlyBall;
	//hsvRoi.copyTo(forGuiShowOnlyBall, justBallMask);
	//cv::imshow("hsvRoi", justBallMask);
	//cv::waitKey(100);
	//cv::imshow("forGuiShowOnlyBall", forGuiShowOnlyBall);
	//cv::waitKey(100);

	return checkHistogramInPic(hsvRoi, justBallMask, minHistogramDiff,
			histResult);

}

bool BallDetector::checkHistogramInPic(cv::Mat &hsvRoi, cv::Mat &justBallMask,
		double minHistogramDiff[3], double histResult[3])
{
	if (params.ball->histList[0].size() < 1
			|| params.ball->histList[1].size() < 1
			|| params.ball->histList[2].size() < 1)
	{
		HAF_ERROR_THROTTLE(1, "Not enough histogram in histList instances.");
		return false;
	}

	cv::Mat hist_base[3];
	if (!calcHist3Channels(hsvRoi, justBallMask, hist_base))
	{
		return false;
	}
	bool histCheck[3];
	for (int cnlIdx = 0; cnlIdx < 3; cnlIdx++)  //HSV channels
	{
		histResult[cnlIdx] = 9;
		histCheck[cnlIdx] = false;
		if (!hist_base[cnlIdx].empty())
		{

			for (std::vector<cv::Mat>::size_type i = 0;
					i < params.ball->histList[cnlIdx].size(); i++)
			{
				double tmpDiff = compareHist(hist_base[cnlIdx],
						params.ball->histList[cnlIdx][i], 3);
				//cout << "Histogram[" << cnlIdx << "] = " << tmpDiff << endl;
				if (tmpDiff < histResult[cnlIdx])
				{
					histResult[cnlIdx] = tmpDiff;
				}
				if (tmpDiff < minHistogramDiff[cnlIdx])
				{
					histCheck[cnlIdx] = true;
				}
			}
		}
		else
		{
			ROS_ERROR("Error in programming");
			return false;
		}
	}

	return histCheck[0] && histCheck[1] && histCheck[2];
}

bool BallDetector::checkDistance_Rec(const cv::Rect &rec,
		CameraProjections &projection, Point2f &realPos)
{

	if (params.ball->distSizeVec.size() < 2)
	{
		HAF_ERROR_THROTTLE(1, "Not enough distanceSize instances.");
		return false;
	}

	double minlen = min(rec.height, rec.width) / 2.;
	double maxlen = max(rec.height, rec.width) / 2.;

	cv::Point center(rec.x + rec.width / 2, rec.y + rec.height / 2);

	if (!projection.GetOnRealCordinate_single(center, realPos))
	{
		ROS_ERROR("Error in programming");
		return false;
	}

	realPos = projection.convertToBallProjection(realPos);
	double distanceToRobot = GetDistance(realPos);

	size_t start = 0;
	size_t end = params.ball->distSizeVec.size() - 1;
	bool rangeFound = false;
	for (size_t i = 0; i < params.ball->distSizeVec.size(); i++)
	{
		if (params.ball->distSizeVec[i].z <= distanceToRobot
				&& i + 1 < params.ball->distSizeVec.size()
				&& params.ball->distSizeVec[i + 1].z > distanceToRobot)
		{
			start = i;
			end = i + 1;
			rangeFound = true;
			break;
		}
	}

	if (!rangeFound)
	{
		if (distanceToRobot < params.ball->distSizeVec[0].z)
		{
			start = 0;
			end = 1;
			rangeFound = true;
		}
		else if (distanceToRobot
				> params.ball->distSizeVec[params.ball->distSizeVec.size() - 1].z)
		{
			start = params.ball->distSizeVec.size() - 2;
			end = params.ball->distSizeVec.size() - 1;
			rangeFound = true;
		}
		else
		{
			ROS_ERROR("Programming Error!");
		}
	}

	double NearMinLen = params.ball->distSizeVec[start].x;
	double NearMaxLen = params.ball->distSizeVec[start].y;
	double NearestDistance = params.ball->distSizeVec[start].z;

	double FarMinLen = params.ball->distSizeVec[end].x;
	double FarMaxLen = params.ball->distSizeVec[end].y;
	double FarestDistance = params.ball->distSizeVec[end].z;

	LineSegment lowerBound(Point2f(NearestDistance, NearMinLen),
			Point2f(FarestDistance, FarMinLen));
	LineSegment higherBound(Point2f(NearestDistance, NearMaxLen),
			Point2f(FarestDistance, FarMaxLen));
	LinearBoundaryChecker checker(lowerBound, higherBound);

	if (!checker.CheckExtrapolation(distanceToRobot, minlen)
			|| !checker.CheckExtrapolation(distanceToRobot, maxlen))
	{

		return false;
	}
	return true;
}

bool BallDetector::Init()
{
	if (!readFromFile<Point3d>(params.ball->distSizeVecPath,
			params.ball->distSizeVec))
	{
		ROS_ERROR("Create or modify %s!", params.ball->distSizeVecPath.c_str());
	}
	std::sort(params.ball->distSizeVec.begin(), params.ball->distSizeVec.end(),
			[](const Point3d& a, const Point3d& b)
			{
				return a.z < b.z;
			});

	for (int i = 0; i < 3; i++)
	{
		if (!readFromFile<Mat>(params.ball->histListcPath[i],
				params.ball->histList[i]))
		{
			ROS_ERROR("Create or modify %s!", params.ball->histListcPath[i].c_str());
		}
	}

	return LoadCascade();
}
bool BallDetector::LoadCascade()
{
	return object_cascade.load(params.configPath + "cascadeBall.xml");
}

