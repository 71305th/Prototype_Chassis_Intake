// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Chassis;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class Burst extends CommandBase {

  DriveSubsystem drive;
  int counter = 0;
  double currentTime;
  double lastTime;
  double iT;

  /** Creates a new Burst. */
  public Burst(DriveSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.setMotor2zero();
    iT = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double dT = Timer.getFPGATimestamp() - iT;
    drive.setLeftSpeed(1);
    drive.setRightSpeed(1);
    System.out.println("sped" + drive.getAverageVelocity());
    System.out.println("dt " + dT);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.setMotor2zero();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (drive.getAverageVelocity() >= 3.6){
      counter++;
    } 
    else counter = 0;
    if (counter > 5) return true;
    else {
      return false;
    }
  }
}
