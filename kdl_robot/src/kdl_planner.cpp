#include "kdl_ros_control/kdl_planner.h"

KDLPlanner::KDLPlanner(double _maxVel, double _maxAcc)
{
    velpref_ = new KDL::VelocityProfile_Trap(_maxVel,_maxAcc);
}

/*
KDLPlanner::KDLPlanner(double _trajDuration, double _accDuration, Eigen::Vector3d _trajInit, Eigen::Vector3d _trajEnd)
{
    trajDuration_ = _trajDuration;
    accDuration_ = _accDuration;
    trajInit_ = _trajInit;
    trajEnd_ = _trajEnd;
}

// constructor to compute circular trajectory
KDLPlanner::KDLPlanner(double _trajDuration, Eigen::Vector3d _trajInit, double _trajRadius){
    trajDuration_ = _trajDuration;
    trajInit_ = _trajInit;
    trajRadius_ = _trajRadius;
}

*/

////////////////////////////////// CONSTRUCTORS
    // constructor to compute linear trajectory with trapezoidal velocity profile
    KDLPlanner::KDLPlanner(double _trajDuration, double _accDuration,
               Eigen::Vector3d _trajInit, Eigen::Vector3d _trajEnd){
                trajDuration_ = _trajDuration;
                accDuration_ = _accDuration;
                trajInit_ = _trajInit;
                trajEnd_ = _trajEnd;
                trajRadius_ = -1;
               }


    // constructor to compute linear trajectory with cubic polinomial profile
    KDLPlanner::KDLPlanner(double _trajDuration,
               Eigen::Vector3d _trajInit, Eigen::Vector3d _trajEnd){
                trajDuration_ = _trajDuration;
                accDuration_ = -1;
                trajInit_ = _trajInit;
                trajEnd_ = _trajEnd;
                trajRadius_ = -1;
               }

    // constructor to compute circular trajectory with trapezoidal velocity profile
    KDLPlanner::KDLPlanner(double _trajDuration, double _accDuration,
               Eigen::Vector3d _trajInit, double _trajRadius){
                trajDuration_ = _trajDuration;
                accDuration_ = _accDuration;
                trajInit_ = _trajInit;
                trajEnd_ = _trajInit;
                trajRadius_ = _trajRadius;
               }
    
    // constructor to compute circular trajectory with cubic polinomial profile
    KDLPlanner::KDLPlanner(double _trajDuration, 
        Eigen::Vector3d _trajInit, double _trajRadius){
                trajDuration_ = _trajDuration;
                accDuration_ = -1;
                trajInit_ = _trajInit;
                trajEnd_ = _trajInit;
                trajRadius_ = _trajRadius;
        }
    //////////////////////////////////////////

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

// trajectory_point KDLPlanner::compute_trajectory(double time)
// {
//   /* trapezoidal velocity profile with accDuration_ acceleration time period and trajDuration_ total duration.
//      time = current time
//      trajDuration_  = final time
//      accDuration_   = acceleration time
//      trajInit_ = trajectory initial point
//      trajEnd_  = trajectory final point */
// 
//   trajectory_point traj;
// 
//   Eigen::Vector3d ddot_traj_c = -1.0/(std::pow(accDuration_,2)-trajDuration_*accDuration_)*(trajEnd_-trajInit_);
// 
//   if(time <= accDuration_)
//   {
//     traj.pos = trajInit_ + 0.5*ddot_traj_c*std::pow(time,2);
//     traj.vel = ddot_traj_c*time;
//     traj.acc = ddot_traj_c;
//   }
//   else if(time <= trajDuration_-accDuration_)
//   {
//     traj.pos = trajInit_ + ddot_traj_c*accDuration_*(time-accDuration_/2);
//     traj.vel = ddot_traj_c*accDuration_;
//     traj.acc = Eigen::Vector3d::Zero();
//   }
//   else
//   {
//     traj.pos = trajEnd_ - 0.5*ddot_traj_c*std::pow(trajDuration_-time,2);
//     traj.vel = ddot_traj_c*(trajDuration_-time);
//     traj.acc = -ddot_traj_c;
//   }
// 
//   return traj;
// 
// }

trajectory_point KDLPlanner::compute_trajectory(double time){

  trajectory_point traj;
  if (trajRadius_ < 0) {
    traj = compute_linear_trajectory(time);
  }
  else if (trajInit_.isApprox(trajEnd_)) {
    traj = compute_circle_trajectory(time);
  }
  else {
    // Constructor error
  }
  return traj;
}


trajectory_point KDLPlanner::compute_circle_trajectory(double time){

    // Debug
    // std::cout << "radius: " << trajRadius_ << std::endl;

    curvilinearAbscissa abscissa;
    if (accDuration_ < 0) {
      abscissa = cubic_polinomial(time);
    }
    else {
      abscissa = trapezoidal_vel(time);
    }

 trajectory_point traj;
 Eigen::Vector3d circleCenter = trajInit_;
 circleCenter(1) += trajRadius_;

 Eigen::Vector3d circle;
 circle << circleCenter(0), 
           circleCenter(1)-trajRadius_*cos(2*M_PI*abscissa.s), 
           circleCenter(2)-trajRadius_*sin(2*M_PI*abscissa.s);

 Eigen::Vector3d circledot;
 circledot << 0, 
              2*M_PI*trajRadius_*abscissa.sdot*sin(2*M_PI*abscissa.s),
             -2*M_PI*trajRadius_*abscissa.sdot*cos(2*M_PI*abscissa.s);
 
 Eigen::Vector3d circleddot;
 circleddot << 0,
               4*pow(M_PI,2)*trajRadius_*std::pow(abscissa.sdot,2)*cos(2*M_PI*abscissa.s) + 2*M_PI*trajRadius_*pow(abscissa.sdot,2)*sin(2*M_PI*abscissa.s),
               4*pow(M_PI,2)*trajRadius_*std::pow(abscissa.sdot,2)*sin(2*M_PI*abscissa.s) - 2*M_PI*trajRadius_*pow(abscissa.sdot,2)*cos(2*M_PI*abscissa.s);
 
 traj.pos = circle; 
 traj.vel = circledot;
 traj.acc = circleddot;

 return traj;
   
}

trajectory_point KDLPlanner::compute_linear_trajectory(double time){

    curvilinearAbscissa abscissa;
    if (accDuration_ < 0) {
      abscissa = cubic_polinomial(time);
    }
    else {
      abscissa = trapezoidal_vel(time);
    }
  trajectory_point traj;
  
  traj.pos = trajInit_ + abscissa.s * (trajEnd_ - trajInit_);
  traj.vel = abscissa.sdot * (trajEnd_ - trajInit_);
  traj.acc = abscissa.sddot * (trajEnd_ - trajInit_);
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

  CoeffsMat << 1,0,0,0,
               0,1,0,0,
               1,trajDuration_,pow(trajDuration_,2),pow(trajDuration_,3),
               0,1,2*trajDuration_,3*pow(trajDuration_,2),
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
  // DEBUG
  // std::cout << "coeffs: " << coeffs << std::endl;
  // std::cout << "s: " << abscissa.s << std::endl;

  return abscissa;
}
