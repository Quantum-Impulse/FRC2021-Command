#include "subsystems/DriveSubsystem.h"

#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>

using namespace DriveConstants;

DriveSubsystem::DriveSubsystem()
    : M_TopLeft{TopLeft ,rev::CANSparkMax::MotorType::kBrushless},
      M_TopRight{TopRight, rev::CANSparkMax::MotorType::kBrushless},
      M_MiddleLeft{MiddleLeft, rev::CANSparkMax::MotorType::kBrushless},
      M_MiddleRight{MiddleRight, rev::CANSparkMax::MotorType::kBrushless},
      M_BottomLeft{BottomLeft, rev::CANSparkMax::MotorType::kBrushless},
      M_BottomRight{BottomRight, rev::CANSparkMax::MotorType::kBrushless},
      m_odometry{ahrs.GetRotation2d()} {
  // Set the distance per pulse for the encoders
  LeftEncoder = M_MiddleLeft.GetEncoder();
  RightEncoder = M_MiddleRight.GetEncoder();
  
  LeftEncoder.SetPositionConversionFactor(kEncoderDistancePerPulse);
  RightEncoder.SetPositionConversionFactor(kEncoderDistancePerPulse);

  ResetEncoders();
}

void DriveSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
  m_odometry.Update(m_gyro.GetRotation2d(),
                    units::meter_t(LeftEncoder.GetDistance()),
                    units::meter_t(RightEncoder.GetDistance()));
}

void DriveSubsystem::ArcadeDrive(double fwd, double rot) {
  m_drive.ArcadeDrive(fwd, rot);
}

void DriveSubsystem::TankDriveVolts(units::volt_t left, units::volt_t right) {
  M_leftMotors.SetVoltage(left);
  M_rightMotors.SetVoltage(-right);
  m_drive.Feed();
}

void DriveSubsystem::ResetEncoders() {
  m_leftEncoder.Reset();
  m_rightEncoder.Reset();
}

double DriveSubsystem::GetAverageEncoderDistance() {
  return (m_leftEncoder.GetDistance() + m_rightEncoder.GetDistance()) / 2.0;
}

frc::Encoder& DriveSubsystem::GetLeftEncoder() { return m_leftEncoder; }

frc::Encoder& DriveSubsystem::GetRightEncoder() { return m_rightEncoder; }

void DriveSubsystem::SetMaxOutput(double maxOutput) {
  m_drive.SetMaxOutput(maxOutput);
}

units::degree_t DriveSubsystem::GetHeading() const {
  return m_gyro.GetRotation2d().Degrees();
}

double DriveSubsystem::GetTurnRate() { return -m_gyro.GetRate(); }

frc::Pose2d DriveSubsystem::GetPose() { return m_odometry.GetPose(); }

frc::DifferentialDriveWheelSpeeds DriveSubsystem::GetWheelSpeeds() {
  return {units::meters_per_second_t(m_leftEncoder.GetRate()),
          units::meters_per_second_t(m_rightEncoder.GetRate())};
}

void DriveSubsystem::ResetOdometry(frc::Pose2d pose) {
  ResetEncoders();
  m_odometry.ResetPosition(pose, m_gyro.GetRotation2d());
}