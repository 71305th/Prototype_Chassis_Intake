// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

  /** Creates a new ExampleSubsystem. */
  public DriveSubsystem() {
    m_motorFrontLeft.setInverted(true);
    m_motorRearLeft.setInverted(true);
    m_leftEncoder.setDistancePerPulse(DriveConstants.kDistancePerPulse);
    m_rightEncoder.setDistancePerPulse(DriveConstants.kDistancePerPulse);
    m_odometry = 
      new DifferentialDriveOdometry(
        m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
    resetEncoders();
  }

  // NEO * 4
  CANSparkMax m_motorFrontLeft = new CANSparkMax(DriveConstants.motorFrontLeft, MotorType.kBrushless);
  CANSparkMax m_motorFrontRight = new CANSparkMax(DriveConstants.motorFrontRight, MotorType.kBrushless);
  CANSparkMax m_motorRearLeft = new CANSparkMax(DriveConstants.motorRearLeft, MotorType.kBrushless);
  CANSparkMax m_motorRearRight = new CANSparkMax(DriveConstants.motorRearRight, MotorType.kBrushless);

  MotorControllerGroup rightGroup = new MotorControllerGroup(m_motorFrontRight, m_motorRearRight);
  MotorControllerGroup leftGroup = new MotorControllerGroup(m_motorFrontLeft, m_motorRearLeft);

  DifferentialDrive m_drive = new DifferentialDrive(leftGroup, rightGroup);

  AHRS m_gyro = new AHRS(Port.kMXP);

  Encoder m_leftEncoder = 
    new Encoder(
      DriveConstants.kLeftEncoderPort[0], DriveConstants.kLeftEncoderPort[1], DriveConstants.kLeftEncoderReversed);
  Encoder m_rightEncoder = 
    new Encoder(
      DriveConstants.kRightEncoderPort[0], DriveConstants.kRightEncoderPort[1], DriveConstants.kLeftEncoderReversed);

  DifferentialDriveOdometry m_odometry = 
    new DifferentialDriveOdometry(
      m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry = 
      new DifferentialDriveOdometry(
        m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void arcadeDrive(double speed, double rotation) {
    m_drive.arcadeDrive(-speed*0.6, rotation*0.7);
  }

  public void tankDrive(double left, double right) {
    m_drive.tankDrive(-left, -right);
  }

  public void setMotor2zero() {
    m_drive.arcadeDrive(0, 0);
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d initialPose) {
    resetEncoders();
    m_odometry.resetPosition(
      m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance(), initialPose);
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   * 
   * <strong> Heading </strong>There is no line break.
   * <strong> Heading </strong>There is no line break.
   * 
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftGroup.setVoltage(leftVolts);
    rightGroup.setVoltage(rightVolts);
    m_drive.feed();
  }

  public double getAverageEncoderDistance() {
    return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
  }

  public Encoder getLeftEncoder() {
    return m_leftEncoder;
  }

  public Encoder getRightEncoder() {
    return m_rightEncoder;
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_gyro.getRate();
  }
}
