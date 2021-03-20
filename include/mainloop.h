#pragma once

#define _USE_MATH_DEFINES

#include <visualizer.h>
#include <robot_independent.h>
#include <ctcr_model.h>
#include <tdcr_model.h>
#include <controller.h>

//stl
#include <ctime>
#include <cmath>
#include <fstream>

//vtk
#include <vtkSmartPointer.h>
#include <vtkCommand.h>
#include <vtkRenderWindowInteractor.h>

//Eigen
#include <Eigen/Dense>

// Class that implements the main simulation loop
class MainLoop : public vtkCommand
{
	private:
		Visualizer* mp_Vis;
		Controller* mp_Controller;
		TDCRModel* mp_TDCR;
		CTCRModel* mp_CTCR;
		double m_timestep;
		int m_loopCount;
		int m_assignment;
		int m_counter;
		std::vector<Eigen::MatrixXd> m_configs;
		bool m_control_loop_active;
		int m_control_scenario;
		Eigen::Matrix4d m_control_target_frame;
		std::vector<Eigen::Matrix4d> m_CTCR_path;
		std::vector<Eigen::Matrix4d> m_TDCR_path;
		double m_control_gain;
		bool m_wdls_jac;
		
	public:
		MainLoop(Visualizer* vis, Controller* con, TDCRModel* tdcr, CTCRModel* ctcr, double timestep, int assignment, int control_scenario);
		~MainLoop();
	
		virtual void Execute(vtkObject *vtkNotUsed(caller), unsigned long eventId, void *vtkNotUsed(callData));

  
    
};
