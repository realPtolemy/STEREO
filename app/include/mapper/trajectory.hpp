#pragma once

#include <map>
#include "transformation.hpp"

/*
Fixa en egen Time klass eller något liknande, kan lägga allt i en `defintions.hpp` fil
*/

template<class DerivedTrajectory>
class Trajectory
{
public:
    typedef std::map<uint64_t, Transformation> PoseMap;

    DerivedTrajectory& derived()
    { return static_cast<DerivedTrajectory&>(*this); }

    Trajectory() {}

    Trajectory(const PoseMap& poses) : poses_(poses) {}

    // Returns T_W_C (mapping points from C to the world frame W)
    bool getPoseAt(const uint64_t& t, Transformation& T) const
    {
        return derived().getPoseAt(t, T);
    }

    void getFirstControlPose(Transformation* T, uint64_t* t) const
    {
        *t = poses_.begin()->first;
        *T = poses_.begin()->second;
    }

    void getLastControlPose(Transformation* T, uint64_t* t) const
    {
        *t = poses_.rbegin()->first;
        *T = poses_.rbegin()->second;
    }

    size_t getNumControlPoses() const
    {
        return poses_.size();
    }

    bool print() const
    {
        size_t control_pose_idx = 0u;
        // for(auto it : poses_)
        // {
        // LOG(INFO) << "--Control pose #" << control_pose_idx++ << ". time = " << it.first;
        // LOG(INFO) << "--T = ";
        // LOG(INFO) << it.second;
        // }
        return true;
    }

    void applyTransformationRight(const Transformation& T)
    {
        for(auto it : poses_)
        {
            poses_[it.first] = it.second * T;
        }
    }

    void applyTransformationLeft(const Transformation& T)
    {
        for(auto it : poses_)
        {
            poses_[it.first] = T * it.second;
        }
    }

protected:
    PoseMap poses_;
};



class LinearTrajectory : public Trajectory<LinearTrajectory>
{
public:

  LinearTrajectory() : Trajectory() {}

  LinearTrajectory(const PoseMap& poses) : Trajectory(poses)
  {
    // CHECK_GE(poses_.size(), 2u) << "At least two poses need to be provided";
  }

  bool getPoseAt(const uint64_t& t, Transformation& T) const
  {
    uint64_t t0_, t1_;
    Transformation T0_, T1_;

    // Check if it is between two known poses
    auto it1 = poses_.upper_bound(t);
    if(it1 ==  poses_.begin())
    {
    //   LOG_FIRST_N(WARNING, 5) << "Cannot extrapolate in the past. Requested pose: "
    //                           << t << " but the earliest pose available is at time: "
    //                           << poses_.begin()->first;
      return false;
    }
    else if(it1 == poses_.end())
    {
    //   LOG_FIRST_N(WARNING, 5) << "Cannot extrapolate in the future. Requested pose: "
    //                           << t << " but the latest pose available is at time: "
    //                           << (poses_.rbegin())->first;
      return false;
    }
    else
    {
      auto it0 = std::prev(it1);
      t0_ = (it0)->first;
      T0_ = (it0)->second;
      t1_ = (it1)->first;
      T1_ = (it1)->second;
    }

    // Linear interpolation in SE(3)
    auto T_relative = T0_.inverse() * T1_;
    auto delta_t = (t - t0_).toSec() / (t1_ - t0_).toSec();
    T = T0_ * Transformation::exp(delta_t * T_relative.log());
    return true;
  }

};
