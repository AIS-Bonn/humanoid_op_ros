//Localization.hpp
// Created on: Apr 20, 2016
//     Author: Hafez Farazi <farazi@ais.uni-bonn.de>
#pragma once
#include <opencv2/opencv.hpp>
#include <math.h>
#include <vision_module/Tools/Parameters.hpp>
#include <vision_module/Tools/General.hpp>
#include <vision_module/Tools/SortFuntions.hpp>
#include <vision_module/Tools/LinearBoundaryChecker.hpp>
#include <vision_module/Tools/LineSegment.hpp>
#include <vision_module/Projections/CameraProjections.hpp>
#include <vision_module/Tools/Kalman.hpp>
#include <vision_module/SoccerObjects/ObstacleDetector.hpp>
#include <gait_msgs/GaitOdom.h>
#include <boost/timer/timer.hpp>
#include <algorithm>    // std::sort
#include <field_model/field_model.h>
#include <tf/transform_broadcaster.h>
#include <math.h>

#include <Eigen/StdVector>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/hyper_graph.h>
#include <g2o/core/solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/solvers/structure_only/structure_only_solver.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam2d/edge_se2.h>
#include <g2o/types/slam2d/vertex_se2.h>

using namespace cv;
using namespace Eigen;
using namespace g2o;

enum LineType
{
	HorUndef,
	HorCenter,
	HorGoal,
	HorGoalNear,
	VerUndef,
	VerLeft,
	VerRight,
	VerLeftNear,
	VerRightNear,
	VerLeftT,
	VerRightT,
	LTRES
};

enum LandmarkType
{
	CenterL = 0, RightL, FrontL, LeftL, BackL
};

const std::string LineTypeName[LTRES] =
{ "HorUndef", "HorCenter", "HorGoal", "HorGoalNear", "VerUndef", "VerLeft",
		"VerRight", "VerLeftNear", "VerRightNear", "VerLeftT", "VerRightT" };
class LineContainer
{
public:
	inline LineContainer(LineSegment _lineTransformed, LineType _type) :
			lineTransformed(_lineTransformed), type(_type)
	{

	}
	LineSegment lineTransformed;
	LineType type;
};

class FeatureContainer
{
public:
	inline FeatureContainer(Point2f _position, string _type) :
			position(_position), type(_type)
	{

	}
	Point2f position;
	string type;
};
/**
* @ingroup VisionModule
*
* @brief For localization in the soccer field
**/
class Localization
{
private:
	inline double getUpdateCoef(double coef, cv::Point2f point)
	{
		double distance = GetDistance(point);

		if (distance > 8)
			return 0;
		if (distance < 3)
			return coef;

		return coef * (1 - (distance / 8));
	}

	inline double getUpdateCoef(double coef, LineSegment line)
	{
		return getUpdateCoef(coef,
				line.GetClosestPointOnLineSegment(cv::Point2d(0, 0)))
				* line.getProbability();
	}

	ros::NodeHandle nodeHandle;
	ros::Subscriber setLoc_sub;
	ros::Subscriber robotTracker_sub;
	ros::Subscriber robotTracker2_sub;
	ros::Subscriber odom_sub;
	Point2d location;
	Point2d locationKalman;
	KalmanFilterC kalmanI;
	CameraProjections *_cameraProjections;
	geometry_msgs::Point robotTrackerLoc;
	float lastOdomX, lastOdomY;
	Point3d globalPos;
	Point3d lastOdom;
	uint32_t lastOdomID;
	Point2f ballPos;
	const field_model::FieldModel* const m_field;
	// TF transforms
	tf::TransformBroadcaster tfBroadcaster;
	tf::StampedTransform tfLocField;
	vector<ObstacleC> *obstacles;
	BlockSolverX::LinearSolverType * linearSolver;
	BlockSolverX* blockSolver;
	OptimizationAlgorithmLevenberg* optimizationAlgorithm;
	int CurrentVertexId;
	int PreviousVertexId;
	int LandmarkCount;
	Point2d odomLastGet;
	bool atLeastOneObservation;
	ros::Time lastSavedNodeTime;
	long unsigned int nodeCounter;
public:
	SparseOptimizer optimizer;

	inline ~Localization()
	{
		optimizer.clear();
		delete optimizationAlgorithm;
	}

	inline Localization(vector<ObstacleC> *obstacles) :
			location(-1.42, 0), locationKalman(location), kalmanI(
					locationKalman), _cameraProjections(
			NULL), lastOdomX(0), lastOdomY(0), globalPos(0, 0, 0), lastOdom(0,
					0, 0), lastOdomID(0), ballPos(0, 0), m_field(
					field_model::FieldModel::getInstance()), obstacles(
					obstacles), CurrentVertexId(0), PreviousVertexId(0), LandmarkCount(
					0), odomLastGet(0, 0), atLeastOneObservation(false), nodeCounter(
					0)
	{

		odom_sub = nodeHandle.subscribe("/gait/odometry", 10,
				&Localization::dead_reckoning_callback, this);
		setLoc_sub = nodeHandle.subscribe<geometry_msgs::Point>(
				"/vision/setLocation", 1, &Localization::setloc_callback, this);

		A = m_field->length();
		B = m_field->width();
		E = m_field->goalAreaDepth();
		F = m_field->goalAreaWidth();
		G = m_field->penaltyMarkerDist();
		H = m_field->centerCircleDiameter();
		D = m_field->goalWidth();
		I = m_field->boundary();
		params.ball->radius.set(m_field->ballDiameter() / 2.);
		A2 = A / 2.;
		B2 = B / 2.;
		H2 = H / 2.;
		A2 = A / 2.;
		B2 = B / 2.;
		E2 = E / 2.;
		F2 = F / 2.;
		G2 = G / 2.;
		H2 = H / 2.;
		D2 = D / 2.;
		I2 = I / 2.;

		// TF transforms
		tfLocField.frame_id_ = "/ego_floor";
		tfLocField.child_frame_id_ = "/loc_field";
		tfLocField.setIdentity();

		lastSavedNodeTime = ros::Time::now();
		// create the linear solver
		linearSolver = new LinearSolverCSparse<BlockSolverX::PoseMatrixType>();
		// create the block solver on the top of the linear solver
		blockSolver = new BlockSolverX(linearSolver);
		//create the algorithm to carry out the optimization
		optimizationAlgorithm = new OptimizationAlgorithmLevenberg(blockSolver);
	}

	inline Point2d getOdometryFromLastGet()
	{
		if (PreviousVertexId > 0) //Not the first iteration
		{
			Point2d res = odomLastGet;
			odomLastGet.x = odomLastGet.y = 0;
			return res;
		}
		else
			return Point2d(0, 0);
	}

	inline void updateVertexIdx()
	{
		if ((ros::Time::now() - lastSavedNodeTime).toSec() >= 0.03)
		{
			nodeCounter++;
			lastSavedNodeTime = ros::Time::now();
			PreviousVertexId = CurrentVertexId;
			CurrentVertexId++;
			if (CurrentVertexId - LandmarkCount >= 100)
			{
				CurrentVertexId = LandmarkCount;
			}

			{
				VertexSE2 * r = new VertexSE2;
				r->setEstimate(Eigen::Vector3d(location.x, location.y, 0));
				r->setFixed(false);
				r->setId(CurrentVertexId);
				if (optimizer.vertex(CurrentVertexId) != NULL)
				{
					optimizer.removeVertex(optimizer.vertex(CurrentVertexId));
				}

				optimizer.addVertex(r);
			}

			{
				EdgeSE2 * e = new EdgeSE2;
				e->vertices()[0] = optimizer.vertex(PreviousVertexId);
				e->vertices()[1] = optimizer.vertex(CurrentVertexId);
				Point2d dead_reck = getOdometryFromLastGet();
				e->setMeasurement(SE2(dead_reck.x, dead_reck.y, 0));
				Matrix3d information;
				information.fill(0.);
				information(0, 0) = 200;
				information(1, 1) = 200;
				information(2, 2) = 1;
				e->setInformation(information);
				optimizer.addEdge(e);
			}
		}
	}
	inline bool addObservation(Point2d observation, double xFasher,
			double yFasher, LandmarkType type)
	{

		{
			EdgeSE2 * e = new EdgeSE2;

			e->vertices()[0] = optimizer.vertex(type);
			e->vertices()[1] = optimizer.vertex(CurrentVertexId);

			switch (type)
			{
			case RightL:
				observation.y += B2;
				break;
			case FrontL:
				observation.x -= A2;
				break;
			case LeftL:
				observation.y -= B2;
				break;
			case BackL:
				observation.x += A2;
				break;
			default:
				break;
			}
			e->setMeasurement(SE2(observation.x, observation.y, 0));
			Matrix3d information;
			information.fill(0.);
			information(0, 0) = xFasher;
			information(1, 1) = yFasher;
			information(2, 2) = 1;
			e->setInformation(information);

			g2o::RobustKernelCauchy* rk = new g2o::RobustKernelCauchy;
			e->setRobustKernel(rk);

			optimizer.addEdge(e);
		}
		atLeastOneObservation = true;
		return true;
	}

	inline Point2f getBall()
	{
		return ballPos;
	}

	inline void setBall(Point2f _in)
	{
		ballPos = _in;
	}

	inline void setloc_callback(const geometry_msgs::Point::ConstPtr& msg)
	{
		ROS_INFO("Set location received.");
		location.x = msg->x;
		location.y = msg->y;
		locationKalman.x = msg->x;
		locationKalman.y = msg->y;

		{
			boundry_n(location.x, -A2 - I, A2 + I);
			boundry_n(location.y, -B2 - I, B2 + I);
			boundry_n(locationKalman.x, -A2 - I, A2 + I);
			boundry_n(locationKalman.y, -B2 - I, B2 + I);
		}
	}

	inline void setremoteloc_callback(
			const geometry_msgs::PointStamped::ConstPtr& msg)
	{
		this->robotTrackerLoc.x = msg->point.x;
		this->robotTrackerLoc.y = msg->point.y;
		this->robotTrackerLoc.z = msg->point.z;
	}

	inline void dead_reckoning_callback(const gait_msgs::GaitOdomConstPtr & msg)
	{
		if (_cameraProjections == NULL)
		{
			return;
		}
		Point3d curOdom;
		curOdom.x = msg->odom2D.x;
		curOdom.y = msg->odom2D.y;
		curOdom.z = msg->odom2D.theta;

		if (lastOdomID == msg->ID)
		{
			Point3d diffOdom = curOdom - lastOdom;
			globalPos += diffOdom;
			Point2d diffOdom2D(diffOdom.x, diffOdom.y);

			Point2d diffOdom2DCancelOdomRotation = RotateAroundPoint(diffOdom2D,
					Radian2Degree(curOdom.z));

			Point2d diffAbsolut = RotateAroundPoint(
					diffOdom2DCancelOdomRotation,
					-Radian2Degree(_cameraProjections->getHeading()));
			location.x += diffAbsolut.x;
			location.y += diffAbsolut.y;
			robotTrackerLoc.x += diffAbsolut.x;
			robotTrackerLoc.y += diffAbsolut.y;
			odomLastGet.x += diffAbsolut.x;
			odomLastGet.y += diffAbsolut.y;

			robotTrackerLoc.z = _cameraProjections->getHeading();
		}

		lastOdom = curOdom;
		lastOdomID = msg->ID;
	}

	inline Point3d GetLocalization()
	{
		Point3d res;
		res.x = location.x;
		res.y = location.y;
		res.z = _cameraProjections->getHeading();
		if (params.loc->useKalman())
		{
			res.x = locationKalman.x;
			res.y = locationKalman.y;
		}
		if (params.loc->forwardRobotTrackerXY())
		{
			res.x = robotTrackerLoc.x;
			res.y = robotTrackerLoc.y;
		}
		if (params.loc->forwardRobotTrackerZ())
		{
			res.z = robotTrackerLoc.z;
		}

		return res;
	}
	void SendTransform(ros::Time now);
	bool Calculate(vector<LineSegment> &, bool circleDetected, const Point2f &,
			const vector<cv::Point2f> &, const Point2d &,
			const vector<Point2f> &, const bool &confiused,
			vector<LineContainer> &, vector<FeatureContainer> &);

	bool Update(CameraProjections &projection);
	bool Init(string rName);
	void InitG2OGraph();

	double A;
	double B;
	double E;
	double F;
	double G;
	double H;
	double D;
	double I;
	double A2;
	double B2;
	double E2;
	double F2;
	double G2;
	double H2;
	double D2;
	double I2;

};
