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
        public static final int motorFrontLeft = 2  ;
        public static final int motorFrontRight = 4;
        public static final int motorRearLeft = 5;
        public static final int motorRearRight = 3;

        // Encoders
        public static final int[] kLeftEncoderPort = {0, 0};
        public static final int[] kRightEncoderPort = {0, 0};
        public static final boolean kLeftEncoderReversed = true;
        public static final boolean kRightEncoderReversed = true;
        public static final double kEncoderCPR = 0;
        public static final double kWheelDiameterMeters = 0;
        public static final double kWheelCircumference = kWheelDiameterMeters*Math.PI;
        public static final double kGearRatio = 0;
        public static final double kDistancePerPulse = kWheelCircumference/kEncoderCPR/kGearRatio;
        // distancePerPulse = gearRatio * wheelPerimeter / EncoderCPR

        // Trajectory Constraints
        public static final int ksVolts = 0;
        public static final int kvVoltSecondsPerMeter = 0;
        public static final int kaVoltSecondsSquaredPerMeter = 0;
        public static final int kPDriveVel = 0;

        // Chassis Related
        public static final double kTrackWidthmeters = 0;
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackWidthmeters);

        // Charge Station
        public static final double kLockPIDLeftkP = 0.1;
        public static final double kLockPIDLeftkI = 0.1;
        public static final double kLockPIDLeftkD = 0.01;
        public static final double kLockPIDLeftiLimit = 0.3;

        public static final double kLockPIDRightkP = 0.1;
        public static final double kLockPIDRightkI = 0.1;
        public static final double kLockPIDRightkD = 0.01;
        public static final double kLockPIDRightiLimit = 0.3;
    }
    
    public final class AutoConstants {
        // Feed forward shit
        public static final int kRamseteB = 0;
        public static final int kRamseteZeta = 0;
        public static final int kMaxSpeedMetersPerSecond = 0;
        public static final int kMaxAccelerationMetersPerSecondSquared = 0;
    }

    public final class IntakeFrontConstants {
        // Left
        public static final int motor1 = 0;
        public static final int motor2 = 0;
        // Right
        public static final int motor3 = 0;
        public static final int motor4 = 0;

        // Intake Wheel Speed 
        public static final int wheelSpeedRPM = 0;
    }

    public final class IntakeRearConstants {
        // Left
        public static final int motor1 = 0;
        public static final int motor2 = 0;
        // Right
        public static final int motor3 = 0;
        public static final int motor4 = 0;

        // Intake Wheel Speed 
        public static final int wheelSpeedRPM = 0;
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
