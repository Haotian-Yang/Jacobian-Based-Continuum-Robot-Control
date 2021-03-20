#pragma once

//stl
#include <cmath>

//Eigen
#include <Eigen/Dense>


#include <robot_independent.h>
#include <ctcr_model.h>
#include <tdcr_model.h>


class Controller
{  
	
	private:
		TDCRModel* mp_TDCR;
		CTCRModel* mp_CTCR; 
		
		
	public:
		Controller(TDCRModel* tdcr, CTCRModel* ctcr);
		~Controller();
		
		
		// This function implements a single control iteration for a three segment TDCR.
		// It implements pose control, meaning it takes a target frame as an input and calculates a step in Q for the TDCR to approach this new frame (in terms of orientation and position).
		// It then applies this step to the TDCR and updates its state for the next control iteration, while returning its new tip frame and shape.  
		//
		// Inputs:
		// T_target					4x4 matrix, specifying the target tip frame for the current control iteration T_sd.
		// wdls_jac					Boolean value, specifying if a damped (singularity robust) Jacobian should be utilized in the control iteration
		// gain						Double value, specifying the proportional gain of the controller
		//
		// Outputs:
		// ee_frame					4x4 matrix, specifying the end-effector frame of the updated TDCR after the control iteration
		// disk_frames				4x4(3*n+1) matrix, storing the disk frames of the updated TDCR after the control iteration
		void execute_tdcr_control_iteration(Eigen::Matrix4d &ee_frame, Eigen::MatrixXd &disk_frames, Eigen::Matrix4d T_target, bool wdls_jac, double gain);
		
		// This function implements a single control iteration for a three tube CTCR.
		// It implements position control, meaning it takes a target frame as an input and calculates a step in Q for the CTCR to approach the new position of this frame (Note: ignoring orientation!).
		// It then applies this step to the CTCR and updates its state for the next control iteration, while returning its new tip frame and shape.  
		//
		// Inputs:
		// T_target					4x4 matrix, specifying the target tip frame for the current control iteration T_sd.
		// wdls_jac					Boolean value, specifying if a damped (singularity robust) Jacobian should be utilized in the control iteration
		// gain						Double value, specifying the proportional gain of the controller
		//
		// Outputs:
		// ee_frame					4x4 matrix, specifying the end-effector frame of the updated CTCR after the control iteration
		// backbone_centerline		4x4(m*n+1) dimensional matrix storing n frames for each of the m subsegments of the updated CTCR after the control iteration
		// tube_ind					Vector with m entries, specifying the outermost tube for each of the m subsegments of the updated CTCR after the control iteration
		void execute_ctcr_control_iteration(Eigen::Matrix4d &ee_frame, Eigen::MatrixXd &backbone_centerline, std::vector<int> &tube_ind, Eigen::Matrix4d T_target, bool wdls_jac, double gain);
};
