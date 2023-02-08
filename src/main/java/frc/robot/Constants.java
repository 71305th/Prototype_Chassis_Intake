// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants {
        // Motors
        public static final int motorFrontLeft = 4;
        public static final int motorFrontRight = 1;
        public static final int motorRearLeft = 2;
        public static final int motorRearRight = 3;

        // Encoders
        public static final int kLeftEncoderPort = 3;
        public static final int kRightEncoderPort = 4;
        public static final double kEncoderCPR = 4096;
        public static final double kWheelDiameterMeters = 0.097;
        public static final double kWheelCircumference = kWheelDiameterMeters * Math.PI;
        public static final double kGearRatio = 0.12;
        public static final double kDistancePerPulse = kGearRatio * kWheelCircumference / kEncoderCPR * 100;
        // distancePerPulse = gearRatio * wheelPerimeter / EncoderCPR

        // Trajectory Constraints
        public static final double ksVolts = 2.4;
        public static final double kvVoltSecondsPerMeter = 7.2;
        public static final double kaVoltSecondsSquaredPerMeter = 0;

        // Chassis Related
        public static final double kTrackWidthmeters = 0.595;
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackWidthmeters);
    }

    public final class PIDConstants {
        // Charge Station
        public static final double kP_Lock = 0.3;
        public static final double kI_Lock = 0.0;
        public static final double kD_Lock = 0.000;
        public static final double iLimit_Lock = 0.36;
    }
    
    public final class AutoConstants {
        // Feed forward shit
        public static final double kMaxSpeedMetersPerSecond = 2.5;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;

        // Positions
        public static final double kPos1X = 1.86;
        public static final double kPos1Y = 5.01;
    }

    public final class PneumaticsConstants {
        // PCM Ports
        public static final int IntakeFrontHorizontalSolenoid = 0;
        public static final int IntakeFrontVerticalSolenoid = 0;
        public static final int IntakeRearHorizontalSolenoid = 0;
        public static final int IntakeRearVerticalSolenoid = 0;
        public static final int Compressor = 0;
    }

    public final class OIConstants {
        // Logitech F310 (X Input Mode)
        // But for this season we'll use XBOX Controller so we won't need this LMAO
        public static final int leftStick_X = 0;
        public static final int leftStick_Y = 1;
        public static final int rightStick_X = 4;
        public static final int rightStick_Y = 5;
        public static final int trigger_L = 2;
        public static final int trigger_R = 3;
        public static final int Btn_A = 1;
        public static final int Btn_B = 2;
        public static final int Btn_X = 3;
        public static final int Btn_Y = 4;
        public static final int Btn_LB = 5;
        public static final int Btn_RB = 6;
        public static final int Btn_LS = 9;  
        public static final int Btn_RS = 10;
    }
}
