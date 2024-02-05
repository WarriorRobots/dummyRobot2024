// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Vars;

public class SwerveModule {
  /** Creates a new SwerveModule. */
  // Last angle of the module. (stored to prevent jittering)
  private Rotation2d lastAngle;
  // Inital Offset of the CanCoder that straightens all modules.
  private double m_angleOffset;
  //private final double m_absoluteEncoderOffset;
  // Motor Controller Declerations
  private WPI_TalonFX m_driveMotor;
  private CANSparkMax m_turnMotor;
  // CAN Coder
  private CANCoder m_absoluteEncoder;
  private RelativeEncoder m_turnEncoder;
  private SparkMaxPIDController m_turnController;
  /**
   * Creates a SwerveModule for the Drivetrain Subsystem.
   * 
   * @param Drive    Motor ID (Int)
   * @param Turn     Motor ID (int)
   * @param Drive    Motor Reversed (Boolean)
   * @param Turn     Motor Reversed (Boolean)
   * @param Drive    Encoder Reversed (Boolean)
   * @param Turn     Motor Encoder Reversed
   * @param CANCODER ID (Int)
   * @param CANCODER Reversed (boolean)
   */
  public SwerveModule(int ID_DRIVE, int ID_TURN, boolean MOTOR_DRIVE_REVERSED, boolean MOTOR_TURN_REVERSED,
  boolean ENCODER_DRIVE_REVERSED, boolean ENCODER_TURN_REVERSED, int ID_CANCODER, boolean CANCODER_REVERSED,
  double angleOffset) {
    // drive motor initalization and config
    m_driveMotor = new WPI_TalonFX(ID_DRIVE);
    m_driveMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.PRIMARY_PID,Constants.MS_TIMEOUT);
    m_driveMotor.setInverted(MOTOR_DRIVE_REVERSED);
    m_driveMotor.setSensorPhase(ENCODER_DRIVE_REVERSED);
    m_driveMotor.config_kP(Constants.PRIMARY_PID, Vars.SWERVE_DRIVEMOTOR_KP);


    // turn motor and encoder inialization
    m_turnMotor = new CANSparkMax(ID_TURN, MotorType.kBrushless);
    m_turnEncoder = m_turnMotor.getEncoder();
    m_turnController = m_turnMotor.getPIDController();
    // Absolute Encoder initalization and config
    m_absoluteEncoder = new CANCoder(ID_CANCODER);
    m_absoluteEncoder.setPositionToAbsolute();
    // Sets the phase of the encoder, 0 is for relay between a check. 0 ms i.e no
    // check.
    m_absoluteEncoder.configSensorDirection(CANCODER_REVERSED, 0);
    // Sets the range for the absolute sensor.
    m_absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    // turn last angle
    lastAngle = getSwerveState().angle;
    // turn motor config
    m_turnMotor.setInverted(MOTOR_TURN_REVERSED);
    m_turnMotor.setCANTimeout(Constants.MS_TIMEOUT);
    m_turnController.setP(Vars.angleKP);
    m_turnController.setI(Vars.angleKI);
    m_turnController.setD(Vars.angleKD);
    m_turnController.setFF(Vars.angleKFF);

    // encoder offsets
    m_angleOffset = angleOffset;
    // sets encoders to 0
    resetEncoders();
    
  }

  /**
   * Returns the Absolute Position of the Module.
   * 
   * @return -180 to 180 degrees. CCW is postive
   */
  // public double getAbsolutePosition() {
  //   return m_absoluteEncoder.getAbsolutePosition();
  // }

  public double getCanCoder() {
    return m_turnEncoder.getPosition();
    //return Rotation2d.fromDegrees(m_turnEncoder.getPosition());
  }



  /**
   * Minimize the change in heading the desired swerve module state would require
   * by potentially
   * reversing the direction the wheel spins. Customized from WPILib's version to
   * include placing
   * in appropriate scope for CTRE onboard control.
   * @param desiredState The desired state.
   * @param currentAngle The current module angle.
   */
  public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
    double targetAngle = placeInAppropriate0To360Scope(currentAngle.getDegrees(), (desiredState.angle.getDegrees()/18));
    double targetSpeed = desiredState.speedMetersPerSecond;
    double delta = (targetAngle - currentAngle.getDegrees())*18;
    if(Math.abs(delta) > 90) {
      targetSpeed = -targetSpeed;
      targetAngle = delta > 90 ? (targetAngle -= 10) : (targetAngle += 10);
    }
    //System.out.println("targetangle " + targetAngle);
    //System.out.println("targetangle " + Rotation2d.fromDegrees(targetAngle));
    // System.out.println("currentangle " + currentAngle.getDegrees());
    //System.out.println("delta " + delta);
    return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
  }

  /**
   * @param scopeReference Current Angle
   * @param newAngle       Target Angle
   * @return Closest angle within scope
   */
  private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
    double lowerBound;
    double upperBound;
    double lowerOffset = scopeReference % 360;
    if (lowerOffset >= 0) {
      lowerBound = scopeReference - lowerOffset;
      upperBound = scopeReference + (360 - lowerOffset);
    } else {
      upperBound = scopeReference - lowerOffset;
      lowerBound = scopeReference - (360 + lowerOffset);
    }
    while (newAngle < lowerBound) {
      newAngle += 360;
    }
    while (newAngle > upperBound) {
      newAngle -= 360;
    }
    if (newAngle - scopeReference > 180) {
      newAngle -= 360;
    } else if (newAngle - scopeReference < -180) {
      newAngle += 360;
    }
    return newAngle;
  }
  /**
   * Turn Motor Position in degrees
   * @return degrees
   */
  public double getTurnDegrees() {
    //return m_absoluteEncoder.getPosition();
    return m_turnEncoder.getPosition();
  }
  /**
   *  Angle of the Module as a Rotation2d
   * @return Rotation2d of the Module.
   */
  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(m_turnEncoder.getPosition());
    //return Rotation2d.fromDegrees(getTurnDegrees());
  }

  /**
   * @return the drive motor's position in inches.
   */
  public double getDrivePosition() {
    return (double) m_driveMotor.getSelectedSensorPosition() / Constants.CLICKS_PER_REV_INTEGRATED
        * Vars.SWERVE_DRIVEMOTOR_GEARING * Math.PI * Vars.WHEEL_DIAMETER;
  }

  /**
   * Converts turn motor native units to degrees.
   * 
   * @param nativeunits of the motor
   * @return the degrees of a given native unit
   */
  public static double toDegreesTurn(double nativeunits) {
    return (double) nativeunits * Vars.SWERVE_TURNMOTOR_GEARING / Constants.CLICKS_PER_REV_INTEGRATED * 360;
  }

  /**
   * Converts degrees to native units from turn motor.
   * 
   * @param degrees degrees
   * @return native units
   */
  public double toNativeTurn(double degrees) {
    return (int) Math.round(degrees / Vars.SWERVE_TURNMOTOR_GEARING * Constants.CLICKS_PER_REV_INTEGRATED / 360.0);
  }

  /**
   * @return The inches/second of the turn encoder.
   */
  public double getTurnVelocity() {
    return m_turnEncoder.getVelocity();
    //return m_absoluteEncoder.getVelocity();
    // return (double) m_turnMotor.getSelectedSensorVelocity() * 10 / Constants.CLICKS_PER_REV_INTEGRATED
    //     * Vars.SWERVE_TURNMOTOR_GEARING * Math.PI * Vars.WHEEL_DIAMETER;
  }

  /**
   * @return The inches/second of the drive encoder.
   */
  public double getDriveVelocity() {
    return (double) m_driveMotor.getSelectedSensorVelocity() * 10 / Constants.CLICKS_PER_REV_INTEGRATED
        * Vars.SWERVE_DRIVEMOTOR_GEARING * Math.PI * Vars.WHEEL_DIAMETER;
  }

  /**
   * Resets drive and turn encoders to 0
   */
  public void resetEncoders() {
    m_driveMotor.setSelectedSensorPosition(0);
    m_turnEncoder.setPosition(0);
    //m_absoluteEncoder.setPosition(0);
  }

  /**
   * 
   * Retrieves swerve state as SwerveModuleState class (module velocity and angle)
   * 
   * @retu
   * rn current swerve state
   */
  public SwerveModuleState getSwerveState() {
    return new SwerveModuleState(getDriveVelocity(), getAngle());
    // return new SwerveModuleState(Units.inchesToMeters(getDriveVelocity()),
    //     new Rotation2d(Units.degreesToRadians(getTurnDegrees())));
  }

  /**
   * Returns the swerve position as SwerveModulePosition class (module position and angle)
   * 
   * @return swerve position
   */
  public SwerveModulePosition getSwerveModulePosition() {
    return new SwerveModulePosition(Units.inchesToMeters(getDrivePosition()),
        new Rotation2d(Units.degreesToRadians(getTurnDegrees())));
  }

  /**
   * Sets the Turn Motor to a given percent
   * 
   * @param percent from -1 to 1.
   */
  public void setTurnPercent(double percent) {
    m_turnMotor.set(percent);
  }
  /** 
   * Sets the turn motor a given angle in degrees. Closed Loop for testing purposes.
   * @param degrees 
   */
  public void setTurnDegrees(double degrees){
    m_turnMotor.set(toNativeTurn(degrees));
  }

  /**
   * Sets the Drive Motor to a given percent
   * 
   * @param percent from -1 to 1.
   */
  public void setDrivePercent(double percent) { 
    m_driveMotor.set(ControlMode.PercentOutput, percent);
  }

  /**
   * 
   * Resets the Turn Motor to the absolute encoder offset.
   */
  public void setModuleToOffset() {
    //m_turnEncoder.setPosition(toNativeTurn( getAbsolutePosition() - m_absoluteEncoderOffset));
    //double absolutePosition = getCanCoder().getDegrees() - angleOffset.getDegrees();
    double absolutePosition = getCanCoder() - m_angleOffset;
    m_turnEncoder.setPosition(absolutePosition);
    //m_absoluteEncoder.setPosition(toNativeTurn(getAbsolutePosition()-m_absoluteEncoderOffset));
  }

  /**
   * Sets the swerve module state. (speed and angle)
   * 
   * @param Desired SwerveModuleState
   */
  public void setSwerveState(SwerveModuleState desiredState) {
    // Removed for testing purposes (used to deaband input)
    SwerveModuleState state;
    state = optimize(desiredState, new Rotation2d(Units.degreesToRadians(getTurnDegrees()/18)));
    //state = optimize(desiredState, getSwerveState().angle);
    setAngle(state);
    setSpeed(state);
  }

  /**
   * Sets the angle of the turn motor from the desired state. Also checks if the
   * state's speed within 1% to prevent jittering of the module.
   * 
   * @param Desired SwerveModuleState
   */
  public void setAngle(SwerveModuleState desiredState) {
    // Prevent rotating module if speed is less then 1%. Prevents Jittering.
    Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Vars.SWERVE_MAX_VELOCITY * 0.01)) ? lastAngle : desiredState.angle;
    m_turnController.setReference(angle.getDegrees(), ControlType.kPosition);
    lastAngle = angle;
  }

  /**
   * Sets the speed of the drive motor from the desired state.
   * 
   * @param Desired SwerveModuleState
   */
  public void setSpeed(SwerveModuleState desiredState) {
    double percentOutput = desiredState.speedMetersPerSecond / Vars.SWERVE_MAX_VELOCITY;
    m_driveMotor.set(ControlMode.PercentOutput, percentOutput);
  }

  /**
   * Stops drive and turn motor.
   */
  public void stop() {
    m_driveMotor.stopMotor();
    m_turnMotor.stopMotor();
  }

}

