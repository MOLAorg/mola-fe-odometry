/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   WheelOdometry.h
 * @brief  Simple SLAM FrontEnd for wheel odometry: creates SE(3) edges from
 * odometry readings.
 * @author Jose Luis Blanco Claraco
 * @date   Jun 13, 2019
 */
#pragma once

#include <mola-kernel/interfaces/FrontEndBase.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
//#include <mrpt/obs/CActionRobotMovement3D.h>

namespace mola
{
/** Simple SLAM FrontEnd for wheel odometry: creates SE(3) edges from odometry
 * readings.
 *
 * \ingroup mola_fe_odometry_grp */
class WheelOdometry : public FrontEndBase
{
    DEFINE_MRPT_OBJECT(WheelOdometry, mola)

   public:
    WheelOdometry();
    virtual ~WheelOdometry() override = default;

    // See docs in base class
    void initialize(const std::string& cfg_block) override;
    void spinOnce() override;
    void onNewObservation(CObservation::Ptr& o) override;

    using OdometryPlanarUncertaintyModel = mrpt::obs::CActionRobotMovement2D::
        TMotionModelOptions::TOptions_GaussianModel;

    /** Re-initializes the front-end */
    void reset();

    struct Parameters
    {
        /** Minimum Euclidean distance (x,y,z) between keyframes inserted into
         * the map [meters]. */
        double min_dist_xyz_between_keyframes{1.5};

        /** Minimum rotation (in 3D space, yaw, pitch,roll, altogether) between
         * keyframes inserted into
         * the map [rad here, degrees in the yaml file]. */
        double min_rotation_between_keyframes{mrpt::DEG2RAD(30.0)};

        /** SE(2) components of odometry Gaussin uncertainty model. Refer to:
         * https://www.mrpt.org/tutorials/programming/odometry-and-motion-models/probabilistic_motion_models/#2_Gaussian_probabilistic_motion_model
         */
        OdometryPlanarUncertaintyModel noise_model_planar;

        /** "Vertical", or "off-plane" uncertainty */
        struct OffPlaneNoiseModel
        {
            double std_z{0.001};
            double std_pitch{mrpt::DEG2RAD(0.01)};
            double std_roll{mrpt::DEG2RAD(0.01)};
        };
        OffPlaneNoiseModel noise_model_vertical;
    };

    /** Algorithm parameters */
    Parameters params_;

   private:
    /** The worker thread pool with 1 thread for processing incomming scans */
    mrpt::WorkerThreadsPool worker_pool_{1};

    /** All variables that hold the algorithm state */
    struct MethodState
    {
        id_t                 last_kf{mola::INVALID_ID};
        mrpt::poses::CPose2D last_absolute_odom{};
        bool                 last_absolute_odom_valid{false};
        mrpt::poses::CPose3D accum_since_last_kf{};
    };

    MethodState state_;

    /** Here happens the actual processing, invoked from the worker thread pool
     * for each incomming observation */
    void doProcessNewObservation(CObservation::Ptr& o);
};

}  // namespace mola
