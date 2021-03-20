#include <controller.h>


Controller::Controller(TDCRModel* tdcr, CTCRModel* ctcr) {
	
	mp_TDCR = tdcr;
	mp_CTCR = ctcr;

}

Controller::~Controller() {

}


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
void Controller::execute_tdcr_control_iteration(Eigen::Matrix4d &ee_frame, Eigen::MatrixXd &disk_frames, Eigen::Matrix4d T_target, bool wdls_jac, double gain)
{
	//Get current frame and configuration of TDCR
	Eigen::Matrix4d T_cur = mp_TDCR->get_ee_frame();
	Eigen::MatrixXd q_cur = mp_TDCR->get_current_config();
	
	//YOUR CODE GOES HERE
	
	//YOUR CODE ENDS HERE
}


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
void Controller::execute_ctcr_control_iteration(Eigen::Matrix4d &ee_frame, Eigen::MatrixXd &backbone_centerline, std::vector<int> &tube_ind, Eigen::Matrix4d T_target, bool wdls_jac, double gain) {
    //Get current frame and configuration of CTCR
    Eigen::Matrix4d T_cur = mp_CTCR->get_ee_frame();
    Eigen::MatrixXd q_cur = mp_CTCR->get_current_config();

    //YOUR CODE GOES HERE
    // Calculate the body twist
    Eigen::MatrixXd V_b = calculate_desired_body_twist(T_target, T_cur);

    // Reduce twist to only consider translation
    Eigen::Matrix<double, 3, 1> V_b_reduced;
    V_b_reduced << V_b.block<3, 1>(3, 0);

    // Apply proportional gain
    V_b_reduced = V_b_reduced * gain;

    // Calculate the body jacobian and reduce to last three rows
    Eigen::MatrixXd J;
    mp_CTCR->get_body_jacobian(J, q_cur);
    Eigen::MatrixXd J_reduced = J.block<3, 6>(3, 0);
    Eigen::MatrixXd J_psudo_inv;

    // Singular Jacobian
    if (wdls_jac) {
        Eigen::Matrix<double, 6, 6> W;
        W.setZero();
        W.diagonal() << 0.001, 0.001, 0.001, 1, 1, 1;
        J_psudo_inv = (J_reduced.transpose() * J_reduced + W).inverse() * J_reduced.transpose();

    }
    else
    {
        J_psudo_inv = J_reduced.transpose() * ((J_reduced * J_reduced.transpose()).inverse());
    }

    Eigen::MatrixXd q_desired = J_psudo_inv * V_b_reduced;
    Eigen::MatrixXd q_new = q_cur;
//    std::cout << "q_cur"<< q_cur <<std::endl;
//    std::cout << "q_desired"<< q_desired <<std::endl;
    q_new.block<3, 1>(3, 0) += q_desired.block<3, 1>(3, 0);
//    std::cout << "q_new"<< q_new <<std::endl;
//	std::cout << body_twist <<std::endl;
//    std::cout << body_twist_reduced <<std::endl;
//    std::cout << "J: "<< J <<std::endl;
//    std::cout << "J_re: "<< J_reduced <<std::endl;
//    std::cout << "J_sudo: "<< J_psudo_inv <<std::endl;
//    std::cout << "q_desired: "<< q_desired <<std::endl;
//    std::cout << "q_new: "<< q_new <<std::endl;
    mp_CTCR->forward_kinematics(ee_frame, backbone_centerline, tube_ind, q_new);


	
	//YOUR CODE ENDS HERE
}
