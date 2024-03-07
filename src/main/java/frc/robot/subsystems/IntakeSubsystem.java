// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.Vars;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  private TalonFX intake_top, intake_bottom;

  public IntakeSubsystem() {
    intake_top = new TalonFX(RobotMap.ID_INTAKE_TOP);
    intake_top.setInverted(Vars.INTAKE_REVERSED);

    intake_bottom = new TalonFX(RobotMap.ID_INTAKE_BOTTOM);
    intake_bottom.setControl(new Follower(intake_top.getDeviceID(), true));

  }

  /**
  * Run the Intake at a percentage.
  * @param top Percent -1.0 to 1.0
  * @param bottom Percent -1.0 to 1.0
  */
  public void setPercentage(double top)
  {
    intake_top.setControl(new DutyCycleOut(top));
  }
  /**
   * The percent output of the motor
   * @return Percent -1.0 to 1.0 that is commanded to the motor
   */
  public double getGain() {
    return intake_top.getDutyCycle().getValueAsDouble();
  }
  /**
  * Stops the Intake motors.
  */
  public void stop() {
    intake_top.stopMotor();
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
