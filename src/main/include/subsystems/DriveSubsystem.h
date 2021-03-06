#pragma once

#include <frc/ADXRS450_Gyro.h>
#include <frc/Encoder.h>
#include <frc/PWMVictorSPX.h>

#include <frc/SpeedControllerGroup.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc2/command/SubsystemBase.h>
#include <units/voltage.h>
#include <frc/SPI.h>

#include <rev/CANSparkMax.h>
#include "AHRS.h"


#include "Constants.h"

class DriveSubsystem : public frc2::SubsystemBase {
 public:
  DriveSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  // Subsystem methods go here.

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  void ArcadeDrive(double fwd, double rot);

  /**
   * Controls each side of the drive directly with a voltage.
   *
   * @param left the commanded left output
   * @param right the commanded right output
   */
  void TankDriveVolts(units::volt_t left, units::volt_t right);

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  void ResetEncoders();

  /**
   * Gets the average distance of the TWO encoders.
   *
   * @return the average of the TWO encoder readings
   */
  double GetAverageEncoderDistance();

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  frc::Encoder& GetLeftEncoder();

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  frc::Encoder& GetRightEncoder();

  /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive
   * more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  void SetMaxOutput(double maxOutput);

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  units::degree_t GetHeading() const;

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  double GetTurnRate();

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  frc::Pose2d GetPose();

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  frc::DifferentialDriveWheelSpeeds GetWheelSpeeds();

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  void ResetOdometry(frc::Pose2d pose);

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  // The motor controllers
  rev::CANSparkMax M_TopLeft ;
  rev::CANSparkMax M_TopRight ;
  rev::CANSparkMax M_MiddleLeft ;
  rev::CANSparkMax M_MiddleRight ;
  rev::CANSparkMax M_BottomLeft ;
  rev::CANSparkMax M_BottomRight ;

  // The motors on the left side of the drive
  frc::SpeedControllerGroup M_leftMotors{M_TopLeft, M_MiddleLeft, M_BottomLeft};

  // The motors on the right side of the drive
  frc::SpeedControllerGroup M_rightMotors{M_TopRight, M_MiddleRight, M_BottomRight};

  // The robot's drive
  frc::DifferentialDrive m_drive{M_leftMotors, M_rightMotors};

/*
  // The left-side drive encoder
  frc::Encoder m_leftEncoder;

  // The right-side drive encoder
  frc::Encoder m_rightEncoder;
*/
rev::CANEncoder LeftEncoder ;
rev::CANEncoder RightEncoder ;

 // The gyro sensor
  frc::ADXRS450_Gyro m_gyro;


  //Nav-XMP board (This contains the Gyro Sensor)
  AHRS ahrs{frc::SPI::Port::kMXP};
  
  // Odometry class for tracking robot pose
  frc::DifferentialDriveOdometry m_odometry;
};