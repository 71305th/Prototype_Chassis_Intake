// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
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

  DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(
    m_gyro.getRotation2d(), getLeftRelativeDistance(), getRightRelativeDistance(), new Pose2d(
      AutoConstants.kPos1X, AutoConstants.kPos1Y, new Rotation2d()));

  /** Creates a new ExampleSubsystem. */
  public DriveSubsystem() {
    m_motorFrontLeft.setInverted(true);
    m_motorRearLeft.setInverted(true);
    m_motorFrontRight.setInverted(false);
    m_motorRearRight.setInverted(false);
    resetEncoders();
    zeroHeading();
    m_odometry = new DifferentialDriveOdometry(
    m_gyro.getRotation2d(), getLeftRelativeDistance(), getRightRelativeDistance(), new Pose2d(
      AutoConstants.kPos1X, AutoConstants.kPos1Y, new Rotation2d()));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // SmartDashboard.putNumber("LeftDis", getLeftRelativeDistance());
    // SmartDashboard.putNumber("RightDis", getRightRelativeDistance());
    // SmartDashboard.putNumber("Heading", getHeading());
    // SmartDashboard.putNumber("PoseX", getPose().getX());
    // SmartDashboard.putNumber("PoseY", getPose().getY());
    // SmartDashboard.putNumber("LeftVel", getLeftVelocity());
    // SmartDashboard.putNumber("RightVel", getRightVelocity());
    // System.out.println(getAverageVelocity());

    // Update the pose
    m_odometry.update(m_gyro.getRotation2d(),
    getLeftRelativeDistance(),
    getRightRelativeDistance());
      
    System.out.println(m_odometry.getPoseMeters().getX() + ", " + m_odometry.getPoseMeters().getY() + ", " + m_odometry.getPoseMeters().getRotation().getDegrees());
    System.out.println("Heading: " + getHeading());
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

  public double getAverageVelocity() {
    return ( getLeftVelocity() + getRightVelocity() ) / 2;
  }

  public void arcadeDrive(double speed, double rotation) {
    m_drive.arcadeDrive(speed, -rotation);
  }

  public void tankDrive(double left, double right) {
    m_drive.tankDrive(left, right);
  }

  public void setLeftSpeed(double speed) {
    leftGroup.set(speed);
  }

  public void setRightSpeed(double speed) {
    rightGroup.set(speed);
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

  // Assuming this method is part of a drivetrain subsystem that provides the necessary methods
  public Command followTrajectoryCommand(String trajName, boolean isFirstPath) {

    PathPlannerTrajectory traj = PathPlanner.loadPath(
      trajName, new PathConstraints(
        AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared));

    return new SequentialCommandGroup(
        new InstantCommand(() -> {
          // Reset odometry for the first path you run during auto
          if(isFirstPath){
              this.resetOdometry(traj.getInitialPose());
          }
        }),
        new PPRamseteCommand(
            traj, 
            this::getPose, // Pose supplier
            new RamseteController(),
            new SimpleMotorFeedforward(DriveConstants.ksVolts, 
              DriveConstants.kvVoltSecondsPerMeter, DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics, // DifferentialDriveKinematics
            this::getWheelSpeeds, // DifferentialDriveWheelSpeeds supplier
            new PIDController(0, 0, 0), // Left controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            new PIDController(0, 0, 0), // Right controller (usually the same values as left controller)
            this::tankDriveVolts, // Voltage biconsumer
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            this // Requires this drive subsystem
        )
    );
  }
}
