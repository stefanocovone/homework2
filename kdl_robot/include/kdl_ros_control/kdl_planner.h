#ifndef KDLPlanner_H
#define KDLPlanner_H

#include <kdl/frames_io.hpp>
#include <kdl/frames.hpp>
#include <kdl/trajectory.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/trajectory_stationary.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl/path_circle.hpp>
#include <kdl/path_roundedcomposite.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/utilities/error.h>
#include <kdl/trajectory_composite.hpp>
#include "Eigen/Dense"
#include <cmath>

struct trajectory_point{
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d vel = Eigen::Vector3d::Zero();
  Eigen::Vector3d acc = Eigen::Vector3d::Zero();
};

struct curvilinearAbscissa{
    double s;
    double sdot;
    double sddot;
};

class KDLPlanner
{

public:

    KDLPlanner(double _maxVel, double _maxAcc);
    void CreateTrajectoryFromFrames(std::vector<KDL::Frame> &_frames,
                                    double _radius, double _eqRadius);
    void createCircPath(KDL::Frame &_F_start,
                        KDL::Vector &_V_centre,
                        KDL::Vector &_V_base_p,
                        KDL::Rotation &_R_base_end,
                        double alpha,
                        double eqradius);

    KDL::Trajectory* getTrajectory();

    ////////////////////////////////// CONSTRUCTORS
    // constructor to compute linear trajectory with trapezoidal velocity profile
    KDLPlanner(double _trajDuration, double _accDuration,
               Eigen::Vector3d _trajInit, Eigen::Vector3d _trajEnd);

    // constructor to compute circular trajectory with trapezoidal velocity profile
    KDLPlanner(double _trajDuration, double _accDuration,
               Eigen::Vector3d _trajInit, double _trajRadius);

    // constructor to compute linear trajectory with cubic polinomial profile
    KDLPlanner(double _trajDuration,
               Eigen::Vector3d _trajInit, Eigen::Vector3d _trajEnd);
    
    // constructor to compute circular trajectory with cubic polinomial profile
    KDLPlanner(double _trajDuration, 
        Eigen::Vector3d _trajInit, double _trajRadius);
    //////////////////////////////////////////

    // trajectory_point compute_trajectory(double time);
    trajectory_point compute_trajectory(double time);


private:



    KDL::Path_RoundedComposite* path_;
    KDL::Path_Circle* path_circle_;
	KDL::VelocityProfile* velpref_;
	KDL::Trajectory* traject_;

    //////////////////////////////////
    double trajDuration_, accDuration_, trajRadius_;
    Eigen::Vector3d trajInit_, trajEnd_;
    trajectory_point p;

    trajectory_point compute_circle_trajectory(double time);
    trajectory_point compute_linear_trajectory(double time);
    
    curvilinearAbscissa trapezoidal_vel(double time);
    curvilinearAbscissa cubic_polinomial(double time);
};

#endif
