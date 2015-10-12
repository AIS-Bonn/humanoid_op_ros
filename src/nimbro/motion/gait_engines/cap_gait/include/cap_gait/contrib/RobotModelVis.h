// ROS visualisation marker helper for RobotModel class
// File: RobotModelVis.h
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef ROBOTMODELVIS_H
#define ROBOTMODELVIS_H

// Includes
#include <cap_gait/contrib/RobotModel.h>
#include <vis_utils/marker_manager.h>

// Namespace
namespace margait_contrib
{
	/**
	* @class RobotModelVis
	*
	* @brief Encapsulates a visualisation of the state of a RobotModel object.
	*
	* The visualisation is based on ROS visualisation markers.
	**/
	class RobotModelVis : public vis_utils::MarkerManager
	{
	public:
		// Enumerations
		enum JointsEnum
		{
			JNT_LSHOULDER,
			JNT_LELBOW,
			JNT_RSHOULDER,
			JNT_RELBOW,
			JNT_LHIP,
			JNT_LKNEE,
			JNT_LANKLE,
			JNT_RHIP,
			JNT_RKNEE,
			JNT_RANKLE,
			JNT_COUNT
		};
		enum LinesEnum
		{
			LIN_TRUNK_A,
			LIN_TRUNK_B,
			LIN_NECK_A,
			LIN_NECK_B,
			LIN_SHOULDER_A,
			LIN_SHOULDER_B,
			LIN_LUARM_A,
			LIN_LUARM_B,
			LIN_LLARM_A,
			LIN_LLARM_B,
			LIN_RUARM_A,
			LIN_RUARM_B,
			LIN_RLARM_A,
			LIN_RLARM_B,
			LIN_HIP_A,
			LIN_HIP_B,
			LIN_LTHIGH_A,
			LIN_LTHIGH_B,
			LIN_RTHIGH_A,
			LIN_RTHIGH_B,
			LIN_LSHANK_A,
			LIN_LSHANK_B,
			LIN_RSHANK_A,
			LIN_RSHANK_B,
			LIN_LFOOTX_A,
			LIN_LFOOTX_B,
			LIN_LFOOTY_A,
			LIN_LFOOTY_B,
			LIN_LFOOTZ_A,
			LIN_LFOOTZ_B,
			LIN_RFOOTX_A,
			LIN_RFOOTX_B,
			LIN_RFOOTY_A,
			LIN_RFOOTY_B,
			LIN_RFOOTZ_A,
			LIN_RFOOTZ_B,
			LIN_SUPP_F_A,
			LIN_SUPP_F_B,
			LIN_SUPP_R_A,
			LIN_SUPP_R_B,
			LIN_SUPP_B_A,
			LIN_SUPP_B_B,
			LIN_SUPP_L_A,
			LIN_SUPP_L_B,
			LIN_SUPN_F_A,
			LIN_SUPN_F_B,
			LIN_SUPN_R_A,
			LIN_SUPN_R_B,
			LIN_SUPN_B_A,
			LIN_SUPN_B_B,
			LIN_SUPN_L_A,
			LIN_SUPN_L_B,
			LIN_COUNT
		};
		
		// Constructor
		RobotModelVis(const RobotModel* model, const std::string& frame) : MarkerManager("~/cap_gait/robot_model", 1)
		 , model(model)
		 , frame(frame)
		 , Com(this, frame)
		 , Head(this, frame)
		 , Chest(this, frame)
		 , Joints(this, frame)
		 , Lines(this, frame)
		 , offsetX(0.0)
		 , offsetY(0.0)
		 , offsetZ(0.0)
		{
			// Create an empty geometry_msgs point
			geometry_msgs::Point pt;
			pt.x = pt.y = pt.z = 0.0;

			// CoM marker
			Com.setColor(0.5, 0.0, 0.5);
			Com.setScale(0.050);
			
			// Head marker
			Head.setColor(0.8, 0.8, 0.8);
			Head.setScale(0.1);
			
			// Chest marker
			Chest.setColor(0.5, 0.0, 0.5);
			Chest.setScale(0.050);

			// Joint markers
			Joints.setType(visualization_msgs::Marker::SPHERE_LIST);
			Joints.marker.points.assign(JNT_COUNT, pt);
			Joints.setColor(0.6, 0.3, 0.0);
			Joints.setScale(0.050);

			// Line markers
			Lines.setType(visualization_msgs::Marker::LINE_LIST);
			Lines.marker.points.assign(LIN_COUNT, pt);
			Lines.setScale(0.010);

			// Line markers - Colours
			visualization_msgs::Marker::_colors_type& cols = Lines.marker.colors;
			visualization_msgs::Marker::_color_type col;
			col.r = 0.0; col.g = 0.0; col.b = 0.0; col.a = 1.0;
			cols.assign(LIN_COUNT, col);
			cols[LIN_TRUNK_A].b = 1.0;
			cols[LIN_TRUNK_B].b = 1.0;
			cols[LIN_NECK_A].r = 1.0;
			cols[LIN_NECK_B].r = 1.0;
			cols[LIN_SHOULDER_A].g = 1.0;
			cols[LIN_SHOULDER_B].g = 1.0;
			cols[LIN_LUARM_A].r = 0.5;
			cols[LIN_LUARM_A].b = 0.5;
			cols[LIN_LUARM_B].r = 0.5;
			cols[LIN_LUARM_B].b = 0.5;
			cols[LIN_LLARM_A].r = 0.5;
			cols[LIN_LLARM_A].b = 0.5;
			cols[LIN_LLARM_B].r = 0.5;
			cols[LIN_LLARM_B].b = 0.5;
			cols[LIN_RUARM_A].r = 0.5;
			cols[LIN_RUARM_A].b = 0.5;
			cols[LIN_RUARM_B].r = 0.5;
			cols[LIN_RUARM_B].b = 0.5;
			cols[LIN_RLARM_A].r = 0.5;
			cols[LIN_RLARM_A].b = 0.5;
			cols[LIN_RLARM_B].r = 0.5;
			cols[LIN_RLARM_B].b = 0.5;
			cols[LIN_HIP_A].g = 1.0;
			cols[LIN_HIP_B].g = 1.0;
			cols[LIN_LTHIGH_A].r = 0.5;
			cols[LIN_LTHIGH_B].r = 0.5;
			cols[LIN_RTHIGH_A].r = 0.5;
			cols[LIN_RTHIGH_B].r = 0.5;
			cols[LIN_LSHANK_A].r = 0.5;
			cols[LIN_LSHANK_B].r = 0.5;
			cols[LIN_RSHANK_A].r = 0.5;
			cols[LIN_RSHANK_B].r = 0.5;
			cols[LIN_LTHIGH_A].b = 0.5;
			cols[LIN_LTHIGH_B].b = 0.5;
			cols[LIN_RTHIGH_A].b = 0.5;
			cols[LIN_RTHIGH_B].b = 0.5;
			cols[LIN_LSHANK_A].b = 0.5;
			cols[LIN_LSHANK_B].b = 0.5;
			cols[LIN_RSHANK_A].b = 0.5;
			cols[LIN_RSHANK_B].b = 0.5;
			cols[LIN_LFOOTX_A].r = 1.0;
			cols[LIN_LFOOTX_B].r = 1.0;
			cols[LIN_LFOOTY_A].g = 1.0;
			cols[LIN_LFOOTY_B].g = 1.0;
			cols[LIN_LFOOTZ_A].b = 1.0;
			cols[LIN_LFOOTZ_B].b = 1.0;
			cols[LIN_RFOOTX_A].r = 1.0;
			cols[LIN_RFOOTX_B].r = 1.0;
			cols[LIN_RFOOTY_A].g = 1.0;
			cols[LIN_RFOOTY_B].g = 1.0;
			cols[LIN_RFOOTZ_A].b = 1.0;
			cols[LIN_RFOOTZ_B].b = 1.0;
			cols[LIN_SUPP_F_A].g = 0.8;
			cols[LIN_SUPP_F_B].g = 0.8;
			cols[LIN_SUPP_R_A].g = 0.8;
			cols[LIN_SUPP_R_B].g = 0.8;
			cols[LIN_SUPP_B_A].g = 0.8;
			cols[LIN_SUPP_B_B].g = 0.8;
			cols[LIN_SUPP_L_A].g = 0.8;
			cols[LIN_SUPP_L_B].g = 0.8;
			cols[LIN_SUPP_F_A].b = 0.8;
			cols[LIN_SUPP_F_B].b = 0.8;
			cols[LIN_SUPP_R_A].b = 0.8;
			cols[LIN_SUPP_R_B].b = 0.8;
			cols[LIN_SUPP_B_A].b = 0.8;
			cols[LIN_SUPP_B_B].b = 0.8;
			cols[LIN_SUPP_L_A].b = 0.8;
			cols[LIN_SUPP_L_B].b = 0.8;
			cols[LIN_SUPN_F_A].b = 1.0;
			cols[LIN_SUPN_F_B].b = 1.0;
			cols[LIN_SUPN_R_A].b = 1.0;
			cols[LIN_SUPN_R_B].b = 1.0;
			cols[LIN_SUPN_B_A].b = 1.0;
			cols[LIN_SUPN_B_B].b = 1.0;
			cols[LIN_SUPN_L_A].b = 1.0;
			cols[LIN_SUPN_L_B].b = 1.0;
		}

		// Initialisation function (allowed to assume that model is constructed)
		void init()
		{
			// Do nothing if we don't have a valid RobotModel object
			if(!model) return;
		}

		// Set visualisation offset
		void setVisOffset(double x, double y, double z)
		{
			offsetX = x;
			offsetY = y;
			offsetZ = z;
		}

		// Update the markers
		void updateMarkers()
		{
			// Do nothing if we don't have a valid RobotModel object
			if(!model) return;
			
			// Retrieve the config parameters
			const cap_gait::CapConfig* config = model->getConfig();
			if(!config) return;

			// Set the model dimensions
			Com.setScale(config->jointDiameter());
			Head.setScale(config->headDiameter());
			Chest.setScale(config->jointDiameter());
			Joints.setScale(config->jointDiameter());
			Lines.setScale(0.25*config->jointDiameter());

			// Retrieve robot model dimensions
			double fh = config->footOffsetZ();
			double fl = config->footLength();
			double fw = config->footWidth();

			// Derive robot model dimensions
			double fwi = 0.50 * fw - config->footOffsetY(); // Distance to outside edge of foot
			double fwo = 0.50 * fw + config->footOffsetY(); // Distance to inside edge of foot
			double flb = 0.50 * fl - config->footOffsetX(); // Distance to back edge of foot
			double flf = 0.50 * fl + config->footOffsetX(); // Distance to front edge of foot

			// Declare variables
			visualization_msgs::Marker::_points_type& jpt = Joints.marker.points;
			visualization_msgs::Marker::_points_type& lpt = Lines.marker.points;

			// Retrieve the frame positions
			qglviewer::Vec comPos = model->base.position();
			qglviewer::Vec neckPos = model->neck.position();
			qglviewer::Vec headPos = model->head.position();
			qglviewer::Vec lShoulderPos = model->lShoulder.position();
			qglviewer::Vec rShoulderPos = model->rShoulder.position();
			qglviewer::Vec lElbowPos = model->lElbow.position();
			qglviewer::Vec rElbowPos = model->rElbow.position();
			qglviewer::Vec lHandPos = model->lHand.position();
			qglviewer::Vec rHandPos = model->rHand.position();
			qglviewer::Vec lHipPos = model->lHip.position();
			qglviewer::Vec rHipPos = model->rHip.position();
			qglviewer::Vec lKneePos = model->lKnee.position();
			qglviewer::Vec rKneePos = model->rKnee.position();
			qglviewer::Vec lAnklePos = model->lAnkle.position();
			qglviewer::Vec rAnklePos = model->rAnkle.position();
			qglviewer::Vec lFootPos = model->lFootFloorPoint.position();
			qglviewer::Vec rFootPos = model->rFootFloorPoint.position();

			// Offset the frame positions
			comPos.x += offsetX; comPos.y += offsetY; comPos.z += offsetZ;
			neckPos.x += offsetX; neckPos.y += offsetY; neckPos.z += offsetZ;
			headPos.x += offsetX; headPos.y += offsetY; headPos.z += offsetZ;
			lShoulderPos.x += offsetX; lShoulderPos.y += offsetY; lShoulderPos.z += offsetZ;
			rShoulderPos.x += offsetX; rShoulderPos.y += offsetY; rShoulderPos.z += offsetZ;
			lElbowPos.x += offsetX; lElbowPos.y += offsetY; lElbowPos.z += offsetZ;
			rElbowPos.x += offsetX; rElbowPos.y += offsetY; rElbowPos.z += offsetZ;
			lHandPos.x += offsetX; lHandPos.y += offsetY; lHandPos.z += offsetZ;
			rHandPos.x += offsetX; rHandPos.y += offsetY; rHandPos.z += offsetZ;
			lHipPos.x += offsetX; lHipPos.y += offsetY; lHipPos.z += offsetZ;
			rHipPos.x += offsetX; rHipPos.y += offsetY; rHipPos.z += offsetZ;
			lKneePos.x += offsetX; lKneePos.y += offsetY; lKneePos.z += offsetZ;
			rKneePos.x += offsetX; rKneePos.y += offsetY; rKneePos.z += offsetZ;
			lAnklePos.x += offsetX; lAnklePos.y += offsetY; lAnklePos.z += offsetZ;
			rAnklePos.x += offsetX; rAnklePos.y += offsetY; rAnklePos.z += offsetZ;
			lFootPos.x += offsetX; lFootPos.y += offsetY; lFootPos.z += offsetZ;
			rFootPos.x += offsetX; rFootPos.y += offsetY; rFootPos.z += offsetZ;

			// CoM marker
			Com.update(comPos.x, comPos.y, comPos.z);
			
			// Head marker
			Head.update(headPos.x, headPos.y, headPos.z);
			
			// Chest marker
			Chest.update(0.5*(lShoulderPos.x + rShoulderPos.x), 0.5*(lShoulderPos.y + rShoulderPos.y), 0.5*(lShoulderPos.z + rShoulderPos.z));

			// Joint markers
			jpt[JNT_LSHOULDER].x = lShoulderPos.x;
			jpt[JNT_LSHOULDER].y = lShoulderPos.y;
			jpt[JNT_LSHOULDER].z = lShoulderPos.z;
			jpt[JNT_RSHOULDER].x = rShoulderPos.x;
			jpt[JNT_RSHOULDER].y = rShoulderPos.y;
			jpt[JNT_RSHOULDER].z = rShoulderPos.z;
			jpt[JNT_LELBOW].x = lElbowPos.x;
			jpt[JNT_LELBOW].y = lElbowPos.y;
			jpt[JNT_LELBOW].z = lElbowPos.z;
			jpt[JNT_RELBOW].x = rElbowPos.x;
			jpt[JNT_RELBOW].y = rElbowPos.y;
			jpt[JNT_RELBOW].z = rElbowPos.z;
			jpt[JNT_LHIP].x = lHipPos.x;
			jpt[JNT_LHIP].y = lHipPos.y;
			jpt[JNT_LHIP].z = lHipPos.z;
			jpt[JNT_RHIP].x = rHipPos.x;
			jpt[JNT_RHIP].y = rHipPos.y;
			jpt[JNT_RHIP].z = rHipPos.z;
			jpt[JNT_LKNEE].x = lKneePos.x;
			jpt[JNT_LKNEE].y = lKneePos.y;
			jpt[JNT_LKNEE].z = lKneePos.z;
			jpt[JNT_RKNEE].x = rKneePos.x;
			jpt[JNT_RKNEE].y = rKneePos.y;
			jpt[JNT_RKNEE].z = rKneePos.z;
			jpt[JNT_LANKLE].x = lAnklePos.x;
			jpt[JNT_LANKLE].y = lAnklePos.y;
			jpt[JNT_LANKLE].z = lAnklePos.z;
			jpt[JNT_RANKLE].x = rAnklePos.x;
			jpt[JNT_RANKLE].y = rAnklePos.y;
			jpt[JNT_RANKLE].z = rAnklePos.z;
			
			// Trunk segments (including hip and shoulder lines)
			lpt[LIN_HIP_A].x = lHipPos.x;
			lpt[LIN_HIP_A].y = lHipPos.y;
			lpt[LIN_HIP_A].z = lHipPos.z;
			lpt[LIN_HIP_B].x = rHipPos.x;
			lpt[LIN_HIP_B].y = rHipPos.y;
			lpt[LIN_HIP_B].z = rHipPos.z;
			lpt[LIN_TRUNK_A].x = comPos.x;
			lpt[LIN_TRUNK_A].y = comPos.y;
			lpt[LIN_TRUNK_A].z = comPos.z;
			lpt[LIN_TRUNK_B].x = neckPos.x;
			lpt[LIN_TRUNK_B].y = neckPos.y;
			lpt[LIN_TRUNK_B].z = neckPos.z;
			lpt[LIN_SHOULDER_A].x = lShoulderPos.x;
			lpt[LIN_SHOULDER_A].y = lShoulderPos.y;
			lpt[LIN_SHOULDER_A].z = lShoulderPos.z;
			lpt[LIN_SHOULDER_B].x = rShoulderPos.x;
			lpt[LIN_SHOULDER_B].y = rShoulderPos.y;
			lpt[LIN_SHOULDER_B].z = rShoulderPos.z;
			
			// Head segments
			lpt[LIN_NECK_A].x = neckPos.x;
			lpt[LIN_NECK_A].y = neckPos.y;
			lpt[LIN_NECK_A].z = neckPos.z;
			lpt[LIN_NECK_B].x = headPos.x;
			lpt[LIN_NECK_B].y = headPos.y;
			lpt[LIN_NECK_B].z = headPos.z;
			
			// Arm segments
			lpt[LIN_LUARM_A].x = lShoulderPos.x;
			lpt[LIN_LUARM_A].y = lShoulderPos.y;
			lpt[LIN_LUARM_A].z = lShoulderPos.z;
			lpt[LIN_LUARM_B].x = lElbowPos.x;
			lpt[LIN_LUARM_B].y = lElbowPos.y;
			lpt[LIN_LUARM_B].z = lElbowPos.z;
			lpt[LIN_LLARM_A].x = lElbowPos.x;
			lpt[LIN_LLARM_A].y = lElbowPos.y;
			lpt[LIN_LLARM_A].z = lElbowPos.z;
			lpt[LIN_LLARM_B].x = lHandPos.x;
			lpt[LIN_LLARM_B].y = lHandPos.y;
			lpt[LIN_LLARM_B].z = lHandPos.z;
			lpt[LIN_RUARM_A].x = rShoulderPos.x;
			lpt[LIN_RUARM_A].y = rShoulderPos.y;
			lpt[LIN_RUARM_A].z = rShoulderPos.z;
			lpt[LIN_RUARM_B].x = rElbowPos.x;
			lpt[LIN_RUARM_B].y = rElbowPos.y;
			lpt[LIN_RUARM_B].z = rElbowPos.z;
			lpt[LIN_RLARM_A].x = rElbowPos.x;
			lpt[LIN_RLARM_A].y = rElbowPos.y;
			lpt[LIN_RLARM_A].z = rElbowPos.z;
			lpt[LIN_RLARM_B].x = rHandPos.x;
			lpt[LIN_RLARM_B].y = rHandPos.y;
			lpt[LIN_RLARM_B].z = rHandPos.z;

			// Leg segments
			lpt[LIN_LTHIGH_A].x = lHipPos.x;
			lpt[LIN_LTHIGH_A].y = lHipPos.y;
			lpt[LIN_LTHIGH_A].z = lHipPos.z;
			lpt[LIN_LTHIGH_B].x = lKneePos.x;
			lpt[LIN_LTHIGH_B].y = lKneePos.y;
			lpt[LIN_LTHIGH_B].z = lKneePos.z;
			lpt[LIN_RTHIGH_A].x = rHipPos.x;
			lpt[LIN_RTHIGH_A].y = rHipPos.y;
			lpt[LIN_RTHIGH_A].z = rHipPos.z;
			lpt[LIN_RTHIGH_B].x = rKneePos.x;
			lpt[LIN_RTHIGH_B].y = rKneePos.y;
			lpt[LIN_RTHIGH_B].z = rKneePos.z;
			lpt[LIN_LSHANK_A].x = lKneePos.x;
			lpt[LIN_LSHANK_A].y = lKneePos.y;
			lpt[LIN_LSHANK_A].z = lKneePos.z;
			lpt[LIN_LSHANK_B].x = lAnklePos.x;
			lpt[LIN_LSHANK_B].y = lAnklePos.y;
			lpt[LIN_LSHANK_B].z = lAnklePos.z;
			lpt[LIN_RSHANK_A].x = rKneePos.x;
			lpt[LIN_RSHANK_A].y = rKneePos.y;
			lpt[LIN_RSHANK_A].z = rKneePos.z;
			lpt[LIN_RSHANK_B].x = rAnklePos.x;
			lpt[LIN_RSHANK_B].y = rAnklePos.y;
			lpt[LIN_RSHANK_B].z = rAnklePos.z;

			// Left foot
			float lMat[3][3] = {{0}};
			model->lFootFloorPoint.orientation().getRotationMatrix(lMat);
			lpt[LIN_LFOOTX_A].x = lFootPos.x - flb * lMat[0][0];
			lpt[LIN_LFOOTX_A].y = lFootPos.y - flb * lMat[1][0];
			lpt[LIN_LFOOTX_A].z = lFootPos.z - flb * lMat[2][0];
			lpt[LIN_LFOOTX_B].x = lFootPos.x + flf * lMat[0][0];
			lpt[LIN_LFOOTX_B].y = lFootPos.y + flf * lMat[1][0];
			lpt[LIN_LFOOTX_B].z = lFootPos.z + flf * lMat[2][0];
			lpt[LIN_LFOOTY_A].x = lFootPos.x - fwi * lMat[0][1];
			lpt[LIN_LFOOTY_A].y = lFootPos.y - fwi * lMat[1][1];
			lpt[LIN_LFOOTY_A].z = lFootPos.z - fwi * lMat[2][1];
			lpt[LIN_LFOOTY_B].x = lFootPos.x + fwo * lMat[0][1];
			lpt[LIN_LFOOTY_B].y = lFootPos.y + fwo * lMat[1][1];
			lpt[LIN_LFOOTY_B].z = lFootPos.z + fwo * lMat[2][1];
			lpt[LIN_LFOOTZ_A].x = lFootPos.x;
			lpt[LIN_LFOOTZ_A].y = lFootPos.y;
			lpt[LIN_LFOOTZ_A].z = lFootPos.z;
			lpt[LIN_LFOOTZ_B].x = lFootPos.x + fh * lMat[0][2];
			lpt[LIN_LFOOTZ_B].y = lFootPos.y + fh * lMat[1][2];
			lpt[LIN_LFOOTZ_B].z = lFootPos.z + fh * lMat[2][2];

			// Right foot
			float rMat[3][3] = {{0}};
			model->rFootFloorPoint.orientation().getRotationMatrix(rMat);
			lpt[LIN_RFOOTX_A].x = rFootPos.x - flb * rMat[0][0];
			lpt[LIN_RFOOTX_A].y = rFootPos.y - flb * rMat[1][0];
			lpt[LIN_RFOOTX_A].z = rFootPos.z - flb * rMat[2][0];
			lpt[LIN_RFOOTX_B].x = rFootPos.x + flf * rMat[0][0];
			lpt[LIN_RFOOTX_B].y = rFootPos.y + flf * rMat[1][0];
			lpt[LIN_RFOOTX_B].z = rFootPos.z + flf * rMat[2][0];
			lpt[LIN_RFOOTY_A].x = rFootPos.x - fwo * rMat[0][1];
			lpt[LIN_RFOOTY_A].y = rFootPos.y - fwo * rMat[1][1];
			lpt[LIN_RFOOTY_A].z = rFootPos.z - fwo * rMat[2][1];
			lpt[LIN_RFOOTY_B].x = rFootPos.x + fwi * rMat[0][1];
			lpt[LIN_RFOOTY_B].y = rFootPos.y + fwi * rMat[1][1];
			lpt[LIN_RFOOTY_B].z = rFootPos.z + fwi * rMat[2][1];
			lpt[LIN_RFOOTZ_A].x = rFootPos.x;
			lpt[LIN_RFOOTZ_A].y = rFootPos.y;
			lpt[LIN_RFOOTZ_A].z = rFootPos.z;
			lpt[LIN_RFOOTZ_B].x = rFootPos.x + fh * rMat[0][2];
			lpt[LIN_RFOOTZ_B].y = rFootPos.y + fh * rMat[1][2];
			lpt[LIN_RFOOTZ_B].z = rFootPos.z + fh * rMat[2][2];

			// Foot boundaries
			double fwp = 0.0, fwn = 0.0;
			qglviewer::Vec* suppFootPos;
			qglviewer::Vec* supnFootPos;
			float (*suppFootMat)[3][3];
			float (*supnFootMat)[3][3];
			if(model->supportLegSign > 0) // Support foot is right foot
			{
				suppFootPos = &rFootPos;
				supnFootPos = &lFootPos;
				suppFootMat = &rMat;
				supnFootMat = &lMat;
				fwp = fwi;
				fwn = fwo;
			}
			else // Support foot is left foot
			{
				suppFootPos = &lFootPos;
				supnFootPos = &rFootPos;
				suppFootMat = &lMat;
				supnFootMat = &rMat;
				fwp = fwo;
				fwn = fwi;
			}
			lpt[LIN_SUPP_F_A].x = suppFootPos->x + flf * (*suppFootMat)[0][0] - fwn * (*suppFootMat)[0][1];
			lpt[LIN_SUPP_F_A].y = suppFootPos->y + flf * (*suppFootMat)[1][0] - fwn * (*suppFootMat)[1][1];
			lpt[LIN_SUPP_F_A].z = suppFootPos->z + flf * (*suppFootMat)[2][0] - fwn * (*suppFootMat)[2][1];
			lpt[LIN_SUPP_R_A].x = suppFootPos->x + flf * (*suppFootMat)[0][0] + fwp * (*suppFootMat)[0][1];
			lpt[LIN_SUPP_R_A].y = suppFootPos->y + flf * (*suppFootMat)[1][0] + fwp * (*suppFootMat)[1][1];
			lpt[LIN_SUPP_R_A].z = suppFootPos->z + flf * (*suppFootMat)[2][0] + fwp * (*suppFootMat)[2][1];
			lpt[LIN_SUPP_B_A].x = suppFootPos->x - flb * (*suppFootMat)[0][0] + fwp * (*suppFootMat)[0][1];
			lpt[LIN_SUPP_B_A].y = suppFootPos->y - flb * (*suppFootMat)[1][0] + fwp * (*suppFootMat)[1][1];
			lpt[LIN_SUPP_B_A].z = suppFootPos->z - flb * (*suppFootMat)[2][0] + fwp * (*suppFootMat)[2][1];
			lpt[LIN_SUPP_L_A].x = suppFootPos->x - flb * (*suppFootMat)[0][0] - fwn * (*suppFootMat)[0][1];
			lpt[LIN_SUPP_L_A].y = suppFootPos->y - flb * (*suppFootMat)[1][0] - fwn * (*suppFootMat)[1][1];
			lpt[LIN_SUPP_L_A].z = suppFootPos->z - flb * (*suppFootMat)[2][0] - fwn * (*suppFootMat)[2][1];
			lpt[LIN_SUPP_F_B] = lpt[LIN_SUPP_R_A];
			lpt[LIN_SUPP_R_B] = lpt[LIN_SUPP_B_A];
			lpt[LIN_SUPP_B_B] = lpt[LIN_SUPP_L_A];
			lpt[LIN_SUPP_L_B] = lpt[LIN_SUPP_F_A];
			lpt[LIN_SUPN_F_A].x = supnFootPos->x + flf * (*supnFootMat)[0][0] - fwp * (*supnFootMat)[0][1];
			lpt[LIN_SUPN_F_A].y = supnFootPos->y + flf * (*supnFootMat)[1][0] - fwp * (*supnFootMat)[1][1];
			lpt[LIN_SUPN_F_A].z = supnFootPos->z + flf * (*supnFootMat)[2][0] - fwp * (*supnFootMat)[2][1];
			lpt[LIN_SUPN_R_A].x = supnFootPos->x + flf * (*supnFootMat)[0][0] + fwn * (*supnFootMat)[0][1];
			lpt[LIN_SUPN_R_A].y = supnFootPos->y + flf * (*supnFootMat)[1][0] + fwn * (*supnFootMat)[1][1];
			lpt[LIN_SUPN_R_A].z = supnFootPos->z + flf * (*supnFootMat)[2][0] + fwn * (*supnFootMat)[2][1];
			lpt[LIN_SUPN_B_A].x = supnFootPos->x - flb * (*supnFootMat)[0][0] + fwn * (*supnFootMat)[0][1];
			lpt[LIN_SUPN_B_A].y = supnFootPos->y - flb * (*supnFootMat)[1][0] + fwn * (*supnFootMat)[1][1];
			lpt[LIN_SUPN_B_A].z = supnFootPos->z - flb * (*supnFootMat)[2][0] + fwn * (*supnFootMat)[2][1];
			lpt[LIN_SUPN_L_A].x = supnFootPos->x - flb * (*supnFootMat)[0][0] - fwp * (*supnFootMat)[0][1];
			lpt[LIN_SUPN_L_A].y = supnFootPos->y - flb * (*supnFootMat)[1][0] - fwp * (*supnFootMat)[1][1];
			lpt[LIN_SUPN_L_A].z = supnFootPos->z - flb * (*supnFootMat)[2][0] - fwp * (*supnFootMat)[2][1];
			lpt[LIN_SUPN_F_B] = lpt[LIN_SUPN_R_A];
			lpt[LIN_SUPN_R_B] = lpt[LIN_SUPN_B_A];
			lpt[LIN_SUPN_B_B] = lpt[LIN_SUPN_L_A];
			lpt[LIN_SUPN_L_B] = lpt[LIN_SUPN_F_A];

			// Manually add the generic markers (GenMarker) for publishing
			add(&Joints);
			add(&Lines);
		}

		// RobotModel object
		const RobotModel* model;

		// Reference frame for visualisation
		const std::string frame;

	private:
		// Markers
		vis_utils::SphereMarker Com;
		vis_utils::SphereMarker Head;
		vis_utils::SphereMarker Chest;
		vis_utils::GenMarker Joints;
		vis_utils::GenMarker Lines;

		// Visualisation offsets
		double offsetX;
		double offsetY;
		double offsetZ;
		
	};
}

#endif /* ROBOTMODELVIS_H */
// EOF