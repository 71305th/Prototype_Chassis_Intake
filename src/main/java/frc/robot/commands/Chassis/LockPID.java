// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Chassis;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

import java.util.concurrent.Delayed;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class LockPID extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final DriveSubsystem drive;

  private PIDController lockPIDLeft = new PIDController(
    DriveConstants.kLockPIDLeftkP, DriveConstants.kLockPIDLeftkI, 
    DriveConstants.kLockPIDLeftkD, DriveConstants.kLockPIDLeftiLimit);

  private PIDController lockPIDRight = new PIDController(
    DriveConstants.kLockPIDRightkP, DriveConstants.kLockPIDRightkI,
    DriveConstants.kLockPIDRightkD, DriveConstants.kLockPIDRightiLimit);

  private Joystick driverJoystick = new Joystick(0);

  private boolean isEnd = false;

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
    drive.setMotor2zero();
    drive.resetEncoders();
    isEnd = false;
    lockPIDLeft.setTolerance(0, 0);
    lockPIDRight.setTolerance(0, 0);
    System.out.println("LockPID Enabled");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(driverJoystick.getRawAxis(OIConstants.leftStick_Y)) > 0.1
    || (Math.abs(driverJoystick.getRawAxis(OIConstants.rightStick_X))) > 0.1) {
      stop();
    }

    double disLeft = drive.getLeftRelativeDistance();
    double disRight = drive.getRightRelativeDistance();

    drive.setLeftSpeed(lockPIDLeft.calculate(disLeft, 0));
    drive.setRightSpeed(lockPIDRight.calculate(disRight, 0));
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

  public void stop() {
    isEnd = true;
  }
}
