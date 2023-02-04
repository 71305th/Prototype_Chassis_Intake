// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Chassis;

import java.util.HashMap;

import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class PathFollowingWithCmd extends CommandBase {

  DriveSubsystem m_drive;
  String pathname;

  FollowPathWithEvents command;

  /** Creates a new PathFollowingWithCmd. */
  public PathFollowingWithCmd(DriveSubsystem drive, String path) {
    // Use addRequirements() here to declare subsystem dependencies.
    pathname = path;
    m_drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // This is just an example event map. It would be better to have a constant, global event map
    // in your code that will be used by all path following commands.
    // HashMap<String, Command> eventMap = new HashMap<>();
    // eventMap.put("marker1", new PrintCommand("Passed marker 1"));
    // eventMap.put("Intake", new IntakeDown());

    // command = new FollowPathWithEvents(
    //   getPathFollowingCommand(examplePath),
    //   examplePath.getMarkers(),
    //   eventMap
    // );
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
