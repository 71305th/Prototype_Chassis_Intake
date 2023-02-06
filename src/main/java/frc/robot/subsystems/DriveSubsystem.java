// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

  CANSparkMax m_motorFrontLeft = new CANSparkMax(DriveConstants.motorFrontLeft, MotorType.kBrushless);
  CANSparkMax m_motorFrontRight = new CANSparkMax(DriveConstants.motorFrontRight, MotorType.kBrushless);
  CANSparkMax m_motorRearLeft = new CANSparkMax(DriveConstants.motorRearLeft, MotorType.kBrushless);
  CANSparkMax m_motorRearRight = new CANSparkMax(DriveConstants.motorRearRight, MotorType.kBrushless);

  MotorControllerGroup rightGroup = new MotorControllerGroup(m_motorFrontRight, m_motorRearRight);
  MotorControllerGroup leftGroup = new MotorControllerGroup(m_motorFrontLeft, m_motorRearLeft);

  DifferentialDrive m_drive = new DifferentialDrive(leftGroup, rightGroup);

  AHRS m_gyro = new AHRS(Port.kMXP);

  WPI_CANCoder m_leftEncoder = new WPI_CANCoder(DriveConstants.kLeftEncoderPort);
  WPI_CANCoder m_rightEncoder = new WPI_CANCoder(DriveConstants.kRightEncoderPort);

  DifferentialDriveOdometry m_odometry = 
    new DifferentialDriveOdometry(
      m_gyro.getRotation2d(), getLeftRelativeDistance(), getRightRelativeDistance());

    /** Creates a new ExampleSubsystem. */
  public DriveSubsystem() {
    m_motorFrontLeft.setInverted(false);
    m_motorRearLeft.setInverted(false);
    m_motorFrontRight.setInverted(true);
    m_motorRearRight.setInverted(true);
    resetEncoders();
    m_gyro.reset();
    m_odometry = 
      new DifferentialDriveOdometry(
        m_gyro.getRotation2d(), getLeftRelativeDistance(), getRightRelativeDistance());
  }

  Joystick js1 = new Joystick(0);

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
      m_odometry = 
      new DifferentialDriveOdometry(
        m_gyro.getRotation2d(), getLeftRelativeDistance(), getRightRelativeDistance());   
      SmartDashboard.putNumber("Left Relative Dis", getLeftRelativeDistance());
      SmartDashboard.putNumber("Right Relative Dis", getRightRelativeDistance());
      SmartDashboard.putNumber("Left Dis Degrees", m_leftEncoder.getPosition());
      SmartDashboard.putNumber("js1 LeftY", js1.getRawAxis(1));
      SmartDashboard.putNumber("js1 RightX", js1.getRawAxis(4));
      SmartDashboard.putNumber("Heading", getHeading());
      SmartDashboard.putNumber("PoseX", getPose().getX());
      SmartDashboard.putNumber("PoseY", getPose().getY());
    }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public double getLeftRelativeDistance() {
    return m_leftEncoder.getPosition() * DriveConstants.kDistancePerPulse;
  }

  public double getRightRelativeDistance() {
    return m_rightEncoder.getPosition() * DriveConstants.kDistancePerPulse;
  }

  public double getleftAbsoluteDistance() {
    return m_leftEncoder.getAbsolutePosition() * DriveConstants.kDistancePerPulse;
  }

  public double getRightAbsoluteDistance() {
    return m_rightEncoder.getAbsolutePosition() * DriveConstants.kDistancePerPulse;
  }

  public double getLeftVelocity() {
    return m_leftEncoder.getVelocity() * DriveConstants.kDistancePerPulse;
  }

  public double getRightVelocity() {
    return m_rightEncoder.getVelocity() * DriveConstants.kDistancePerPulse;
  }

  public void arcadeDrive(double speed, double rotation) {
    m_drive.arcadeDrive(-speed*0.7, rotation*0.85);
  }

  public void tankDrive(double left, double right) {
    m_drive.tankDrive(-left * 0.7, -right * 0.7);
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
      m_gyro.getRotation2d(), getLeftRelativeDistance(), getRightRelativeDistance(), initialPose);
  }

  public void resetEncoders() {
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightVelocity());
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftGroup.setVoltage(leftVolts);
    rightGroup.setVoltage(rightVolts);
    m_drive.feed();
  }

  public double getAverageEncoderRelativeDistance() {
    return (getLeftRelativeDistance() + getRightRelativeDistance()) / 2.0;
  }

  public CANCoder getLeftEncoder() {
    return m_leftEncoder;
  }

  public CANCoder getRightEncoder() {
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
