/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.DashboardContainer;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.Vars;
import frc.robot.DashboardContainer.TabsIndex;

/**
 * Contains the a single wheel shooter with a PID to command the velocity of the shooter.
 */
public class ShooterSubsystem extends SubsystemBase {
  
  private TalonFX shooter_top;
  private TalonFX shooter_bottom;
  
  /** Number of encoder clicks per every revolution of the encoder */
  static final int CLICKS_PER_REV = 2048; // https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-resolution
  /** Typical motor output as percent */
  static final double ESTIMATED_VOLTAGE = .83;
  /** Velocity of shooter in native units per 100ms at typical motor output (at the encoder) */
  static final int NATIVE_ESTIMATED_VELOCITY = 18600;
  
  private static ShuffleboardTab driverTab = DashboardContainer.getInstance().getTab(TabsIndex.kDriver);
  private static GenericEntry rpmConfig = driverTab.add("RPM Command", Vars.SHOOTER_DEFAULT).withPosition(0, 0).getEntry();
  private static GenericEntry voltageConfig = driverTab.add("Percentage Command", ESTIMATED_VOLTAGE).withPosition(0, 1).getEntry();
  
  /**
   * Instantiates new subsystem; make ONLY ONE.
	 * <p>
	 * <code> public static final ShooterSubsystem shooter = new
	 * ShooterSubsystem();
	 */
  public ShooterSubsystem()
  {
    shooter_top = new TalonFX(RobotMap.ID_SHOOTER_TOP);
    
    shooter_top.setInverted(Vars.SHOOTER_TOP_REVERSED);    

    var shooterFeedback = new FeedbackConfigs();
    shooterFeedback.FeedbackRemoteSensorID = Constants.PRIMARY_PID; // Sensor ID
    shooter_top.getConfigurator().apply(shooterFeedback);

    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = Vars.SHOOTER_KP;
    shooter_top.getConfigurator().apply(slot0Configs);   
    
    shooter_bottom = new TalonFX(RobotMap.ID_SHOOTER_BOTTOM);
    shooter_bottom.setControl(new Follower(shooter_top.getDeviceID(), true));
    //shooter_bottom.setInverted(Vars.SHOOTER_RIGHT_REVERSED);
    
  }

  /**
   * Run the shooter motor at a percent from -1 to 1.
   * @param voltage Percent from -1 to 1.
   */
  public void setVoltage(double voltage)
  {
    shooter_top.setControl(new DutyCycleOut(voltage));
  }

  /**
   * Run the shooter motor at an RPM.
   * @param rpm RPM of the shooter flywheel. (Not the motor or encoder.)
   */
  public void setRPM(double rpm)
  {
    shooter_top.setControl(new DutyCycleOut(toNative(rpm)));
  }

  /**
   * @return The percent the talon is commanding to the motor.
   */
  public double getGain()
  {
    return shooter_top.getDutyCycle().getValueAsDouble();
  }

  /**
   * @return The RPM of the shooter flywheel. (Not the motor or encoder.)
   */
  public double getRPM()
  {
    // (native / 100ms) * (600ms / m) * (rev/native) = rev / m
    return toRPM(shooter_top.getPosition().getValueAsDouble());
  }

  /**
   * @return the raw encoder value.
   */
  public double getEncoder() {
    return shooter_top.getPosition().getValueAsDouble();
  }

  /**
   * @return the velocity of the motor in 
   */
  public double getEncVelocity() {
    return shooter_top.getPosition().getValueAsDouble();
  }

  /**
   * Converts from native units per 100ms to RPM.
   * @param native_units Native units / 100ms
   * @return RPM
   */
  public static double toRPM(double native_units)
  { 
    return ((native_units * 600) / CLICKS_PER_REV);
  }

  /**
   * @return The value from the dashboard for how fast the shooter should be going in RPM.
   */
  public double getCommandedRPM() {
    return rpmConfig.getDouble(Vars.SHOOTER_DEFAULT);
  }

  /**
   * @return The value from the dashboard for how fast the shooter should be based on a percentage.
   */
  public double getCommandedPercent() {
    return voltageConfig.getDouble(0);
  }
  
  /**
   * Converts from RPM to native units per 100ms.
   * @param rpm RPM
   * @return Native units / 100ms
   */
  public static double toNative(double rpm)
  { 
    return ((rpm / 600) * CLICKS_PER_REV);
  }

  public void stop() {
    shooter_top.stopMotor();
  }

  @Override
  public void periodic() {
    putDashboard();
  }

  public void putDashboard() {
    switch (DashboardContainer.getInstance().getVerbosity()) {
      case 5:
        SmartDashboard.putNumber("Shooter/Encoder", getEncoder());
        SmartDashboard.putNumber("Shooter/Native units per 100ms", getEncVelocity());
      case 4:
      case 3:
        SmartDashboard.putNumber("Shooter/Gain", getGain());
      case 2:
        SmartDashboard.putNumber("Shooter/RPM", getRPM());
      case 1:
        break;
      default:
        break;
    }
  }
}