// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.print.CancelablePrintJob;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.Vars;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  // Motor Declaration
  private CANSparkMax arm_left, arm_right;
  private RelativeEncoder m_armEncoder;
  private SparkPIDController m_armController;


  public ArmSubsystem() {
    // Configuration
    arm_left = new CANSparkMax(RobotMap.ID_ARM_LEFT, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    m_armEncoder = arm_left.getEncoder();
    m_armController = arm_left.getPIDController();

    // turn motor config
    arm_left.setInverted(Vars.LEFT_ARM_REVERSED);
    arm_left.setCANTimeout(Constants.MS_TIMEOUT);
    m_armController.setP(Vars.angleKP);
    m_armController.setI(Vars.angleKI);
    m_armController.setD(Vars.angleKD);
    m_armController.setFF(Vars.angleKFF);

    arm_right = new CANSparkMax(RobotMap.ID_ARM_RIGHT, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    arm_right.follow(arm_left);
    arm_right.setInverted(Vars.RIGHT_ARM_REVERSED);

    // encoder offsets
    resetEncoders();

  }

  /**
   * Runs the Arms at a percent from -1 to 1
   * 
   * @param percent percent from -1 to 1
   */
  public void setPercent(double percent) {
    arm_left.set(percent);
  }

  // Sets the arm to the given unbounded angle (NO SAFETY)
  public void setArmAngleUnbounded(double degrees) {
    arm_left.set(toNative(degrees));
  }

  public void setAngleUnbounded(double degrees) {
    setArmAngleUnbounded(degrees);
  }

  // Sets the arm to the given bounded angle
  public void setAngleBounded(double degrees) {
    setArmAngleBounded(degrees);
  }

  /**
   * Sets the angle of the bounded arm (in degrees)
   * 
   * @param degrees
   */
  public void setArmAngleBounded(double degrees) {
    if (degrees < Vars.ARM_MIN_ANGLE) {
      setArmAngleUnbounded(Vars.ARM_MIN_ANGLE);
    } else if (degrees > Vars.ARM_MAX_ANGLE) {
      setArmAngleUnbounded(Vars.ARM_MAX_ANGLE);
    } else {
      setArmAngleUnbounded(degrees);
    }
  }

  public double getArmEnc() {
    return m_armEncoder.getPosition();
  }

  public double getArmAngle() {
    return toDegrees(getArmEnc());
  }


  public double toNative(double degrees) {
    return Math.round(degrees / Vars.ARM_GEARING * Constants.CLICKS_PER_REV_INTEGRATED / (360.0*12));
  }

  public double toDegrees(double nativeUnits) {
    return nativeUnits * Vars.ARM_GEARING / Constants.CLICKS_PER_REV_INTEGRATED * (360.0*12);
  }

  public void resetEncoders() {
    m_armEncoder.setPosition(0);
  }

  public void stop() {
    arm_left.stopMotor();
  }

  @Override
  public void periodic() {
  SmartDashboard.putNumber("Encoder value", getArmEnc());
  SmartDashboard.putNumber("Arm Angle", getArmAngle());
  }

}
