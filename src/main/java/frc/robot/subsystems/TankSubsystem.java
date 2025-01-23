// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class TankSubsystem extends SubsystemBase {
  /** Creates a new TankSubsystem. */

  private TalonFX frontleft, frontright, rearleft, rearright;
  private TalonFX frontleftturn, frontrightturn, rearleftturn, rearrightturn;

  public TankSubsystem() {
    frontleft = new TalonFX(RobotMap.ID_FRONTLEFT_DRIVE);
    frontright = new TalonFX(RobotMap.ID_FRONTRIGHT_DRIVE);
    rearleft = new TalonFX(RobotMap.ID_REARLEFT_DRIVE);
    rearright = new TalonFX(RobotMap.ID_REARRIGHT_DRIVE);

    frontright.setInverted(false);
    rearright.setInverted(false);

    frontleft.setNeutralMode(NeutralModeValue.Brake);
    frontright.setNeutralMode(NeutralModeValue.Brake);
    rearleft.setNeutralMode(NeutralModeValue.Brake);
    rearright.setNeutralMode(NeutralModeValue.Brake);

    frontleftturn = new TalonFX(RobotMap.ID_FRONTLEFT_TURN);
    frontrightturn = new TalonFX(RobotMap.ID_FRONTRIGHT_TURN);
    rearleftturn = new TalonFX(RobotMap.ID_REARLEFT_TURN);
    rearrightturn = new TalonFX(RobotMap.ID_REARRIGHT_TURN);

    frontleftturn.setNeutralMode(NeutralModeValue.Brake);
    frontrightturn.setNeutralMode(NeutralModeValue.Brake);
    rearleftturn.setNeutralMode(NeutralModeValue.Brake);
    rearrightturn.setNeutralMode(NeutralModeValue.Brake);
  }

  public void runLeft(double percent) {
    frontleft.setControl(new DutyCycleOut(percent));
    rearleft.setControl(new DutyCycleOut(percent));
  }

  public void runRight(double percent) {
    frontright.setControl(new DutyCycleOut(percent));
    rearright.setControl(new DutyCycleOut(percent));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
