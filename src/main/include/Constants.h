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

constexpr auto ks = 0.938_V;
constexpr auto kv = 0.719 * 1_V * 1_s / 1_m;
constexpr auto ka = 0.0297 * 1_V * 1_s * 1_s / 1_m;
// Example value only - as above, this must be tuned for your drive!
constexpr double kPDriveVel = 8.5;

constexpr auto kTrackwidth = 1.442_m;
extern const frc::DifferentialDriveKinematics kDriveKinematics;
constexpr auto kMaxSpeed = 3_mps;
constexpr auto kMaxAcceleration = 3_mps_sq;
// Reasonable baseline values for a RAMSETE follower in units of meters and
// seconds
constexpr double kRamseteB = 2;
constexpr double kRamseteZeta = 0.7;
