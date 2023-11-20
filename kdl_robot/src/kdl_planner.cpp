#include "kdl_ros_control/kdl_planner.h"

KDLPlanner::KDLPlanner(double _maxVel, double _maxAcc)
{
    velpref_ = new KDL::VelocityProfile_Trap(_maxVel,_maxAcc);
}

KDLPlanner::KDLPlanner(double _trajDuration, double _accDuration, Eigen::Vector3d _trajInit, Eigen::Vector3d _trajEnd)
{
    trajDuration_ = _trajDuration;
    accDuration_ = _accDuration;
    trajInit_ = _trajInit;
    trajEnd_ = _trajEnd;
}

void KDLPlanner::CreateTrajectoryFromFrames(std::vector<KDL::Frame> &_frames,
                                            double _radius, double _eqRadius
                                            )
{
    path_ = new KDL::Path_RoundedComposite(_radius,_eqRadius,new KDL::RotationalInterpolation_SingleAxis());

    for (unsigned int i = 0; i < _frames.size(); i++)
    {
        path_->Add(_frames[i]);
    }
    path_->Finish();

    velpref_->SetProfile(0,path_->PathLength());
    traject_ = new KDL::Trajectory_Segment(path_, velpref_);
}

void KDLPlanner::createCircPath(KDL::Frame &_F_start,
                                KDL::Vector &_V_centre,
                                KDL::Vector& _V_base_p,
                                KDL::Rotation& _R_base_end,
                                double alpha,
                                double eqradius
                                )
{
    KDL::RotationalInterpolation_SingleAxis* otraj;
    otraj = new KDL::RotationalInterpolation_SingleAxis();
    otraj->SetStartEnd(_F_start.M,_R_base_end);
    path_circle_ = new KDL::Path_Circle(_F_start,
                                        _V_centre,
                                        _V_base_p,
                                        _R_base_end,
                                        alpha,
                                        otraj,
                                        eqradius);
    velpref_->SetProfile(0,path_circle_->PathLength());
    traject_ = new KDL::Trajectory_Segment(path_circle_, velpref_);
}

KDL::Trajectory* KDLPlanner::getTrajectory()
{
	return traject_;
}

trajectory_point KDLPlanner::compute_trajectory(double time)
{
  /* trapezoidal velocity profile with accDuration_ acceleration time period and trajDuration_ total duration.
     time = current time
     trajDuration_  = final time
     accDuration_   = acceleration time
     trajInit_ = trajectory initial point
     trajEnd_  = trajectory final point */

  trajectory_point traj;

  Eigen::Vector3d ddot_traj_c = -1.0/(std::pow(accDuration_,2)-trajDuration_*accDuration_)*(trajEnd_-trajInit_);

  if(time <= accDuration_)
  {
    traj.pos = trajInit_ + 0.5*ddot_traj_c*std::pow(time,2);
    traj.vel = ddot_traj_c*time;
    traj.acc = ddot_traj_c;
  }
  else if(time <= trajDuration_-accDuration_)
  {
    traj.pos = trajInit_ + ddot_traj_c*accDuration_*(time-accDuration_/2);
    traj.vel = ddot_traj_c*accDuration_;
    traj.acc = Eigen::Vector3d::Zero();
  }
  else
  {
    traj.pos = trajEnd_ - 0.5*ddot_traj_c*std::pow(trajDuration_-time,2);
    traj.vel = ddot_traj_c*(trajDuration_-time);
    traj.acc = -ddot_traj_c;
  }

  return traj;

}



curvilinearAbscissa KDLPlanner::trapezoidal_vel(double time){

curvilinearAbscissa abscissa;

double scddot = -1.0/(std::pow(accDuration_,2)-trajDuration_*accDuration_);

if (time >= 0 && time <= accDuration_) {
    abscissa.s = 0.5*scddot*std::pow(time,2);
    abscissa.sdot = scddot*time;
    abscissa.sddot = scddot;
} else if (time > accDuration_ && time <= trajDuration_-accDuration_) {
    abscissa.s = 0.5*scddot*(time-accDuration_/2);
    abscissa.sdot = 0.5*scddot;
    abscissa.sddot = 0;
} else if (time > (trajDuration_-accDuration_) && time <= trajDuration_) {
    abscissa.s = 1 - 0.5*scddot*std::pow(trajDuration_-time,2);
    abscissa.sdot = scddot*(trajDuration_-time);
    abscissa.sddot = -scddot;
} else { 
  // error
}

  return abscissa;
}








curvilinearAbscissa KDLPlanner::cubic_polinomial(double time) {

  curvilinearAbscissa abscissa;

  static bool coeffsComputed = false;
  static Eigen::Matrix4d CoeffsMat;
  static Eigen::Vector4d Boundaries;
  static Eigen::Vector4d coeffs;

  CoeffsMat << 0,0,0,1,   
              0,0,1,0,  
              pow(trajDuration_,3),pow(trajDuration_,2),trajDuration_,1,
              3*pow(trajDuration_,2),2*trajDuration_,1,0;
  Boundaries << 0,0,1,0;

  if (!coeffsComputed){
    coeffs = CoeffsMat.colPivHouseholderQr().solve(Boundaries);
    coeffsComputed = true;
  }


  if (time >= 0 && time <= trajDuration_) {
  abscissa.s = coeffs(3)*pow(time,3) + coeffs(2)*pow(time,2) + coeffs(1)*time + coeffs(0);
  abscissa.sdot = 3*coeffs(3)*pow(time,2) + 2*coeffs(2)*time + coeffs(1);
  abscissa.sddot = 6*coeffs(3)*time + 2*coeffs(2);
  }
  else {
    // error
  } 

  return abscissa;
}
