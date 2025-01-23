/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

//import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.DashboardContainer;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
//import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.GenericEntry;
//import edu.wpi.first.networktables.NetworkTableEntry;
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
    shooter_bottom = new TalonFX(RobotMap.ID_SHOOTER_BOTTOM);
    
    shooter_top.setInverted(Vars.SHOOTER_TOP_REVERSED);
    shooter_bottom.setInverted(Vars.SHOOTER_BOTTOM_REVERSED);
    
    var shooterFeedback = new FeedbackConfigs();
    shooterFeedback.FeedbackRemoteSensorID = Constants.PRIMARY_PID; // Sensor ID
    shooter_top.getConfigurator().apply(shooterFeedback);
    shooter_bottom.getConfigurator().apply(shooterFeedback);

    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = Vars.SHOOTER_KP;
    shooter_top.getConfigurator().apply(slot0Configs);   
    shooter_bottom.getConfigurator().apply(slot0Configs);

    var currentLimitConfigs = new CurrentLimitsConfigs();
    currentLimitConfigs.StatorCurrentLimitEnable = true;
    currentLimitConfigs.StatorCurrentLimit = 30;
    shooter_top.getConfigurator().apply(currentLimitConfigs);
    shooter_bottom.getConfigurator().apply(currentLimitConfigs);

    shooter_bottom = new TalonFX(RobotMap.ID_SHOOTER_BOTTOM);
    shooter_bottom.setControl(new Follower(shooter_top.getDeviceID(), false));

  }

  /**
   * Run the shooter motor at a percent from -1 to 1.
   * @param voltage Percent from -1 to 1.
   */
  public void setVoltage(double voltage)
  {
    shooter_top.setControl(new DutyCycleOut(voltage));
    shooter_bottom.setControl(new DutyCycleOut(voltage));
  }

  /**
   * Run the shooter motor at an RPM.
   * @param rpm RPM of the shooter flywheel. (Not the motor or encoder.)
   */
  public void setRPM(double rpm)
  {
    shooter_top.setControl(new VelocityDutyCycle(toNative(rpm)));
    shooter_bottom.setControl(new VelocityDutyCycle(toNative(rpm*0.5)));
    //shooter_top.setControl(new VelocityVoltage(toNative(rpm*60)));
    //shooter_top.setControl(new DutyCycleOut(toNative(rpm)));
  }

  /**
   * @return The percent the talon is commanding to the motor.
   */
  public double getGain()
  {
    return shooter_top.getDutyCycle().getValueAsDouble();
  }

  /**
   * @return The percent the talon is commanding to the motor.
   */
  public double getGain_bottom()
  {
    return shooter_bottom.getDutyCycle().getValueAsDouble();
  }

  /**
   * @return The RPM of the shooter flywheel. (Not the motor or encoder.)
   */
  public double getRPM()
  {
    return getEncVelocity()*60;
  }

  /**
   * @return the raw encoder value.
   */
  public double getEncoder() {
    return shooter_top.getPosition().getValueAsDouble();
  }

  /**
   * @return the raw encoder value.
   */
  public double getEncoder_bottom() {
    return shooter_bottom.getPosition().getValueAsDouble();
  }

  /**
   * @return the velocity of the motor in 
   */
  public double getEncVelocity() {
    return shooter_top.getVelocity().getValueAsDouble();
  }

  /**
   * @return the velocity of the motor in 
   */
  public double getEncVelocity_bottom() {
    return shooter_bottom.getVelocity().getValueAsDouble();
  }


  /**
   * @return The value from the dashboard for how fast the shooter should be going in RPM.
   */
  public double getCommandedRPM() {
    return rpmConfig.getDouble(Vars.SHOOTER_DEFAULT);
  }
  
  public double getCommandedAmpRPM() {
    return rpmConfig.getDouble(Vars.SHOOTER_AMP);
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
    return rpm/60;
  }

  public void stop() {
    shooter_top.stopMotor();
    shooter_bottom.stopMotor();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter/Encoder", getEncoder());
    SmartDashboard.putNumber("Shooter/Encoder", getEncoder_bottom());
    SmartDashboard.putNumber("Shooter/Encover RPM", getEncVelocity());
    SmartDashboard.putNumber("Shooter/Encover RPM", getEncVelocity_bottom());
    SmartDashboard.putNumber("Shooter/Gain", getGain());
    SmartDashboard.putNumber("Shooter/Gain", getGain_bottom());
    SmartDashboard.putNumber("Shooter/RPM", getRPM());
  }

}