// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeFrontSubsystem;
import frc.robot.subsystems.IntakeRearSubsystem;

public class IntakeCmd extends CommandBase {

  IntakeFrontSubsystem intakeFront;
  IntakeRearSubsystem intakeRear;
  String side;
  String action;

  /** Creates a new IntakeCmd. */
  public IntakeCmd(IntakeFrontSubsystem front, IntakeRearSubsystem rear, String side, String action) {
    // Use addRequirements() here to declare subsystem dependencies.
    intakeFront = front;
    intakeRear = rear;
    this.side = side;
    this.action = action;
    addRequirements(front, rear);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch (action) {
      case "Front":
        
        break;
      case "Rear":
        break;
      default:
        break;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public enum IntakeActions {
    UP,
    DOWN,
    OPEN,
    CLOSE,
    RUN,
  }

  public enum IntakeSide {
    FRONT,
    REAR,
  }
}
