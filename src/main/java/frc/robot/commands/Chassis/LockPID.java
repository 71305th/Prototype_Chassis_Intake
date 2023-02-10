// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Chassis;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.DriveSubsystem;

import javax.sound.sampled.TargetDataLine;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class LockPID extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final DriveSubsystem drive;

  private boolean isEnd = false;
  private double lasttime = 0;
  private Joystick driverJoystick = new Joystick(OIConstants.driverJoystick);

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public LockPID(DriveSubsystem drive) {
    this.drive = drive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.resetEncoders();
    drive.setMotor2zero();
    isEnd = false;
    // lockPIDLeft.setTolerance(0, 0);
    // lockPIDRight.setTolerance(0, 0);
    System.out.println("LockPID Enabled");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if ( Math.abs(driverJoystick.getRawAxis(OIConstants.leftStick_Y)) > 0.1 ) stop();
    if ( Math.abs(driverJoystick.getRawAxis(OIConstants.rightStick_X)) > 0.1 ) stop();

    System.out.println("Enter");

    double disLeft = drive.getLeftRelativeDistance();
    double disRight = drive.getRightRelativeDistance();

    double LeftOutput = -PID( PIDConstants.kP_Lock, PIDConstants.kI_Lock, PIDConstants.kD_Lock, PIDConstants.iLimit_Lock, disLeft, 0 );
    double RightOutput = -PID( PIDConstants.kP_Lock, PIDConstants.kI_Lock, PIDConstants.kD_Lock, PIDConstants.iLimit_Lock, disRight, 0 );

    drive.setLeftSpeed( LeftOutput );
    drive.setRightSpeed( RightOutput );

    // System.out.print("LeftOutput : ");
    // System.out.println(LeftOutput);

    // System.out.print("RightOutput : ");
    // System.out.println(RightOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    stop();
    System.out.println("LockPID Disabled");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isEnd;
  }

  public double PID ( double kP, double kI, double kD, double iLimit, double ctrPos, double target ) {
    double output = 0;
    double error = target - ctrPos;
    double error_sum = 0;
    double time = Timer.getFPGATimestamp();
    double deltaT = time - lasttime;
    double deltaError = error / deltaT;

    if( ctrPos < target + iLimit || ctrPos > target - iLimit ) error_sum += error;
    
    output = kP * error + kI * error_sum + kD * deltaError;
    lasttime = time;

    return output;
  }

  public void stop() {
    isEnd = true;
  }
}
