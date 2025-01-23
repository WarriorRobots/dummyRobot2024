// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

//import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.TankSubsystem;

public class TankDrive extends Command {
  /** Creates a new TankDrive. */

  TankSubsystem tank;
  private TalonFX frontleft, frontright, rearleft, rearright;
  private DoubleSupplier yInputL, yInputR;

  public TankDrive(TankSubsystem tank, DoubleSupplier yInputL, DoubleSupplier yInputR) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.tank = tank;
    addRequirements(tank);
    frontleft = new TalonFX(RobotMap.ID_FRONTLEFT_DRIVE);
    frontright = new TalonFX(RobotMap.ID_FRONTRIGHT_DRIVE);
    rearleft = new TalonFX(RobotMap.ID_REARLEFT_DRIVE);
    rearright = new TalonFX(RobotMap.ID_REARRIGHT_DRIVE);

    this.yInputL = yInputL;
    this.yInputR = yInputR;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    tank.runLeft(yInputL.getAsDouble());
    tank.runRight(yInputR.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    frontleft.stopMotor();
    frontright.stopMotor();
    rearleft.stopMotor();
    rearright.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
