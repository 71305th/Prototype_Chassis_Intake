// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.Intake.IntakeEnums.IntakeAction;
import frc.robot.commands.Intake.IntakeEnums.IntakeSide;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCmd extends CommandBase {

  IntakeSubsystem intake;
  IntakeSide side;
  IntakeAction action;

  /** Creates a new IntakeCmd. */
  public IntakeCmd(IntakeSubsystem intake, IntakeSide side, IntakeAction action) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.side = side;
    this.action = action;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.enableCompressor();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (side) {
      case FRONT:
        switch (action) {
          case UPDOWN:
            intake.frontDown();
            break;
          case OPENCLOSE:
            intake.frontOpen();
            break;
        }
        System.out.println("IntakeCmd executed.");
        break;
      case REAR:
        switch (action) {
          case UPDOWN:
            intake.RearDown();
            break;
          case OPENCLOSE:
            intake.RearOpen();
            break;
        }
        System.out.println("IntakeCmd executed.");
        break;
      default:
        System.out.println("IntakeCmd failed.");
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    switch (side) {
      case FRONT:
        switch (action) {
          case UPDOWN:
            intake.frontUp();
            break;
          case OPENCLOSE:
            intake.frontClose();
            break;
        }
        System.out.println("IntakeCmd executed.");
        break;
      case REAR:
        switch (action) {
          case UPDOWN:
            intake.RearUp();
            break;
          case OPENCLOSE:
            intake.RearClose();
            break;
        }
        System.out.println("IntakeCmd executed.");
        break;
      default:
        System.out.println("IntakeCmd failed.");
        break;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
