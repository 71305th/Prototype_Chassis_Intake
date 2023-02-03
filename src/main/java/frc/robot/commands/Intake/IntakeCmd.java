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
    switch (side) {
      case FRONT:
        switch (action) {
          case UP:
          case DOWN:
          case OPEN:
          case CLOSE:
            intake.enableCompressor();
            break;
          case RUN:
          case STOP:
            intake.frontStop();
          default:
            System.out.println("IntakeCmd action failed: init");
            break;
        }
        break;
      case REAR:
        switch (action) {
          case UP:
          case DOWN:
          case OPEN:
          case CLOSE:
            intake.enableCompressor();
            break;
          case RUN:
          case STOP:
            intake.RearStop();
          default:
            System.out.println("IntakeCmd action failed: init");
            break;
        }
        break;
      default:
        break;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (side) {
      case FRONT:
        switch (action) {
          case UP:
            intake.frontUp();
          case DOWN:
            intake.frontDown();
          case OPEN:
            intake.frontOpen();
          case CLOSE:
            intake.enableCompressor();
            break;
          case RUN:
            intake.frontRun();
          case STOP:
            intake.frontStop();
          default:
            System.out.println("IntakeCmd action failed: executing");
            break;
        }
        break;
      case REAR:
        switch (action) {
          case UP:
            intake.RearUp();
          case DOWN:
            intake.RearDown();
          case OPEN:
            intake.RearOpen();
          case CLOSE:
            intake.enableCompressor();
            break;
          case RUN:
            intake.RearRun();
          case STOP:
            intake.RearStop();
          default:
            System.out.println("IntakeCmd action failed: executing");
            break;
        }
        break;
      default:
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    switch (side) {
      case FRONT:
        switch (action) {
          case UP:
          case DOWN:
          case OPEN:
          case CLOSE:
            break;
          case RUN:
          case STOP:
            intake.frontStop();
          default:
            System.out.println("IntakeCmd action failed: end");
            break;
        }
        break;
      case REAR:
        switch (action) {
          case UP:
          case DOWN:
          case OPEN:
          case CLOSE:
            break;
          case RUN:
          case STOP:
            intake.RearStop();
          default:
            System.out.println("IntakeCmd action failed: end");
            break;
        }
        break;
      default:
        break;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
