#include <tdcr_model.h>


TDCRModel::TDCRModel(std::array<double,3> length, int disks_per_seg, std::array<double,3> pradius_disks, Eigen::Matrix4d base_frame) {
	
	m_length[0]         = length[0];
    m_length[1]         = length[1];
    m_length[2]         = length[2];
    m_disks_per_seg		= disks_per_seg;
    m_pradius_disks[0]  = pradius_disks[0];
    m_pradius_disks[1]  = pradius_disks[1];
    m_pradius_disks[2]  = pradius_disks[2];
    m_base_frame		= base_frame;
    
    
    m_current_config.resize(9,1);
    m_current_config.setZero();
}

TDCRModel::~TDCRModel() {

}



// This function should implement the forward kinematics of a tendon driven continuum robot (i.e. mapping tendon lengths changes in joint space to the robot's end-effector and disk frames)
// Inputs:
// q						9x1 matrix/vector holding the tendon displacement (changes in tendon lengths) for each tendon.
//							The first three tendons belong to segment one etc (3 Tendons per 3 Segments).
//							A negative value indicates that the tendon is shortened (i.e. pulled).
//
// Outputs:
// ee_frame					4x4 matrix storing the end-effector frame resulting from the actuation q.
// disk_frames				4x4(3*n+1) dimensional matrix storing the frames for each of the n disks for each segment.
//							The leftmost 4x4 block should store the initial base/disk frame of the robot.
// boolean return value		True if kinematics have been calculated successfully, false if not.
//							Also return false, if the tendon length constraints are invalidated (the sum of tendon length changes in each segment has to equal zero).
bool TDCRModel::forward_kinematics(Eigen::Matrix4d &ee_frame, Eigen::MatrixXd &disk_frames, Eigen::MatrixXd q)
{
	//YOUR CODE GOES HERE
	return false; // Delete this line once you start coding
	
	
	//YOUR CODE ENDS HERE
	
	//Setting the member variables accordingly
	m_ee_frame = ee_frame;
	m_disk_frames = disk_frames;
	m_current_config = q;
	
	return true;

}


// This function should calculate and return the body Jacobian of a tendon driven continuum robot using a simple finite differences approach
// Inputs:
// q						9x1 matrix/vector holding the tendon displacement (changes in tendon lengths) for each tendon.
//							The first three tendons belong to segment one etc (3 Tendons per 3 Segments).
// J						6x6 body Jacobian matrix, relating joint space velocities with end-effector twist (beware the tendon constraints are manually resolved to reduce Q to six dimensions, representing the robot's DoF)
//							The first three rows correspond to the rotational part of a twist, while the last three rows correspond to the translational part
//
// Outputs:
// boolean return value		Return false, if if the tendon length constraints are invalidated.
//							Return true otherwise (and proceed to calculate and return J).
bool TDCRModel::get_body_jacobian(Eigen::MatrixXd &J, Eigen::MatrixXd q)
{
	//Resize J and set to zero
	J.setZero(6,6);
	
	//Evaluate at current configuration q
	Eigen::Matrix4d init_ee_frame;
	Eigen::MatrixXd init_disk_frames;
	Eigen::MatrixXd init_q = q;
	
	if(!forward_kinematics(init_ee_frame, init_disk_frames, q))
	{
		//Return false if joint value constraints are not met
		return false;
	}
	
	
	
	//Calculate the Body Jacobian using Finite Differences here (YOUR CODE GOES HERE)
	
	
	
	//YOUR CODE ENDS HERE

	//Setting the member variables accordingly
	m_ee_frame = init_ee_frame;
	m_disk_frames = init_disk_frames;
	m_current_config = init_q;
	
	return true;
}

Eigen::MatrixXd TDCRModel::get_current_config()
{
	return m_current_config;
}

Eigen::Matrix4d TDCRModel::get_ee_frame()
{
	return m_ee_frame;	
}

Eigen::MatrixXd TDCRModel::get_disk_frames()
{
	return m_disk_frames;	
}

Eigen::Matrix4d TDCRModel::get_base_frame()
{
	return m_base_frame;	
}

