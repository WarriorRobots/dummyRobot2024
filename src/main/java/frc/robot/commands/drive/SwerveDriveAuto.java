// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

//import java.util.function.BooleanSupplier;
//import java.util.function.DoubleSupplier;

//import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
//import frc.robot.Vars;
import frc.robot.KnightsSwerve.DriveManipulation;
//import frc.robot.subsystems.DrivetrainSubsystem;

public class SwerveDriveAuto extends Command {
  /** Creates a new SwerveDriveCommand. */
  private final DriveManipulation drive;
  private final double yInput, xInput, rotInput;

  public SwerveDriveAuto(DriveManipulation drive, double xInput, double yInput, double rotInput) {
    this.drive = drive;
    this.xInput = xInput;
    this.yInput = -yInput;
    this.rotInput = -rotInput;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.setNewCenterState(xInput, yInput, rotInput);
    drive.runToState(Robot.precisionMode);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
