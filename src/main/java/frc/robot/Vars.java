// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;

/** All variables reside here. */
public class Vars {
  // Front Left Module Phases
  public static final boolean FRONTLEFT_DRIVE_MOTOR_REVERSED = false;
  public static final boolean FRONTLEFT_TURN_MOTOR_REVERSED = true;

  public static final boolean FRONTLEFT_DRIVE_ENCODER_REVERSED = true;
  public static final boolean FRONTLEFT_TURN_ENCODER_REVERSED = false;
  public static final boolean FRONTLEFT_CANCODER_REVERSED = false;

  // Front Right Module Phases
  public static final boolean FRONTRIGHT_DRIVE_MOTOR_REVERSED = true;
  public static final boolean FRONTRIGHT_TURN_MOTOR_REVERSED = true;

  public static final boolean FRONTRIGHT_DRIVE_ENCODER_REVERSED = true;
  public static final boolean FRONTRIGHT_TURN_ENCODER_REVERSED = false;
  public static final boolean FRONTRIGHT_CANCODER_REVERSED = false;

  // Rear Left Module Phases
  public static final boolean REARLEFT_DRIVE_MOTOR_REVERSED = false;
  public static final boolean REARLEFT_TURN_MOTOR_REVERSED = true;

  public static final boolean REARLEFT_DRIVE_ENCODER_REVERSED = true;
  public static final boolean REARLEFT_TURN_ENCODER_REVERSED = true;
  public static final boolean REARLEFT_CANCODER_REVERSED = false;

  // Rear Right Module Phases
  public static final boolean REARRIGHT_DRIVE_MOTOR_REVERSED = true;
  public static final boolean REARRIGHT_TURN_MOTOR_REVERSED = true;
    
  public static final boolean REARRIGHT_DRIVE_ENCODER_REVERSED = true;
  public static final boolean REARRIGHT_TURN_ENCODER_REVERSED = true;
  public static final boolean REARRIGHT_CANCODER_REVERSED = false;

  // Swerve Drive Values
  public static final double SWERVE_TURNMOTOR_KP = 0.7;
  public static final double SWERVE_DRIVEMOTOR_KP = 0.2;

  /* Angle Motor PID Values */
  public static final double angleKP = 0.7;
  public static final double angleKI = 0.00;
  public static final double angleKD = 0.03;
  public static final double angleKFF = 0.00;

  public static final double SWERVE_DRIVEMOTOR_GEARING = 1 / 6.75;
  public static final double SWERVE_TURNMOTOR_GEARING = 1 / 21.42;

  public static final double SWERVE_INPUTDEADBAND = 0.02;
  public static double SWERVE_MAX_VELOCITY = 3; // M/S
  public static final double WHEEL_DIAMETER = 4; // inches (above average)
  public static final double WHEEL_Circumference = WHEEL_DIAMETER * 3.14;

  public static final double SWERVE_ANGLE_TOLERANCE = 5;

  /** The locations of the wheels relative to the physical center of the robot. */
  // Postive X is forward, Positive Y is to the left.
  public static final Translation2d[] swerve = new Translation2d[] {
    // Front Left
    new Translation2d(Units.inchesToMeters(11.25), Units.inchesToMeters(11.25)),
    // Front Right
    new Translation2d(Units.inchesToMeters(11.25), Units.inchesToMeters(-11.25)),
    // Rear Left
    new Translation2d(Units.inchesToMeters(-11.25), Units.inchesToMeters(11.25)),
    // Rear Right
    new Translation2d(Units.inchesToMeters(-11.25), Units.inchesToMeters(-11.25)) 
    };
  //public static final double[] absoluteEncoderOffsets = {-37, -166, -48, 21};
  //public static final double[] absoluteEncoderOffsets = {-10.8, 3.3, -8.3, 6.2};
  public static final double[] absoluteEncoderOffsets = {0, 0, 0, 0};
  //public static final Rotation2d angleOffset = Rotation2d.fromDegrees(327.48046875);
  //public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);
  // Swerve Drive Kinematics
  public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(swerve);
  // Auto Constraints
  public static final double AUTO_MAX_M_PER_S = 1; // in/s
  public static final double AUTO_MAX_M_PER_S_SQUARED = 1; // in/s^2

  //Arm Pneumatics
  public static final int PNEUMATIC_LOOP_COUNT = 10; // # of loops the pneumatics are commanded

  // Intake
  public static final boolean INTAKE_REVERSED = true;
  public static final double INTAKE_FORWARD = 0.8;
  public static final double INTAKE_BACKWARD = -1;

}
