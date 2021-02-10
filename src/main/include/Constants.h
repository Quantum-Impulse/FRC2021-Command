// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/trajectory/constraint/DifferentialDriveKinematicsConstraint.h>
#include <units/acceleration.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <wpi/math>

using namespace units::math;

namespace DriveConstants {
     constexpr int TopLeft = 1;
     constexpr int TopRight = 2;
     constexpr int MiddleLeft = 3;
     constexpr int MiddleRight = 4;
     constexpr int BottomLeft = 5;
     constexpr int BottomRight = 6;


    constexpr auto ks = 0.938_V;
    constexpr auto kv = 0.719 * 1_V * 1_s / 1_m;
    constexpr auto ka = 0.0297 * 1_V * 1_s * 1_s / 1_m;

    // Example value only - as above, this must be tuned for your drive!
    constexpr double kPDriveVel = 8.5; // Has not been tune yet!!!!!!

    constexpr auto kTrackwidth = 1.442_m;
    extern const frc::DifferentialDriveKinematics kDriveKinematics;

    constexpr int kEncoderCPR = 42;

    // Reasonable baseline values for a RAMSETE follower in units of meters and
    // seconds
    constexpr double kRamseteB = 2;
    constexpr double kRamseteZeta = 0.7;
} // namespace DriveConstants

namespace AutoConstants {
    constexpr auto kMaxSpeed = 3_mps;
    constexpr auto kMaxAcceleration = 3_mps_sq;
} //namespace AutoConstants

namespace OIConstants {
    constexpr int DriverController =  0;
} // namespace IOConstants