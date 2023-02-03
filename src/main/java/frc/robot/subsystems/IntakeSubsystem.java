// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeFrontConstants;
import frc.robot.Constants.IntakeRearConstants;
import frc.robot.Constants.PneumaticsConstants;

public class IntakeSubsystem extends SubsystemBase {

  // Front
  CANSparkMax m_MotorFrontLeft = new CANSparkMax(IntakeFrontConstants.motorLeft, MotorType.kBrushless);
  CANSparkMax m_MotorFrontRight = new CANSparkMax(IntakeFrontConstants.motorRight, MotorType.kBrushless);
  Solenoid m_FrontSolenoidVertical = new Solenoid(
    PneumaticsModuleType.CTREPCM, PneumaticsConstants.IntakeFrontVerticalSolenoid);
  Solenoid m_FrontSolenoidHorizontal = new Solenoid(
    PneumaticsModuleType.CTREPCM, PneumaticsConstants.IntakeFrontHorizontalSolenoid);
  
  // Rear
  CANSparkMax m_MotorRearLeft = new CANSparkMax(IntakeRearConstants.motorLeft, MotorType.kBrushless);
  CANSparkMax m_MotorRearRight = new CANSparkMax(IntakeRearConstants.motorRight, MotorType.kBrushless);
  Solenoid m_RearSolenoidVertical = new Solenoid(
    PneumaticsModuleType.CTREPCM, PneumaticsConstants.IntakeRearVerticalSolenoid);
  Solenoid m_RearSolenoidHorizontal = new Solenoid(
    PneumaticsModuleType.CTREPCM, PneumaticsConstants.IntakeRearHorizontalSolenoid);
  
  Compressor m_compressor = new Compressor(PneumaticsConstants.Compressor, PneumaticsModuleType.CTREPCM);

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() { 
    m_MotorFrontLeft.follow(m_MotorFrontRight, true);
    m_MotorRearLeft.follow(m_MotorRearRight, true);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void enableCompressor() {
    m_compressor.enableDigital();
  }

  public void frontDown() {
    m_FrontSolenoidVertical.set(false);
  }

  public void frontUp() {
    m_FrontSolenoidVertical.set(true);
  }

  public void frontOpen() {
    m_FrontSolenoidHorizontal.set(true);
  }

  public void frontClose() {
    m_FrontSolenoidHorizontal.set(false);
  }

  public void frontRun() {
    m_MotorFrontRight.set(IntakeFrontConstants.wheelSpeedVolt);
  }

  public void frontStop() {
    m_MotorFrontRight.stopMotor();
    m_MotorFrontLeft.stopMotor();
  }

  public void RearDown() {
    m_RearSolenoidVertical.set(false);
  }

  public void RearUp() {
    m_RearSolenoidVertical.set(true);
  }

  public void RearOpen() {
    m_RearSolenoidHorizontal.set(true);
  }

  public void RearClose() {
    m_RearSolenoidHorizontal.set(false);
  }

  public void RearRun() {
    m_MotorRearLeft.set(IntakeRearConstants.wheelSpeedVolt);
  }

  public void RearStop() {
    m_MotorRearLeft.stopMotor();
    m_MotorRearRight.stopMotor();
  }
}
