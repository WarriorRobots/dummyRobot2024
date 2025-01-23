// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

//import java.util.function.DoubleSupplier;

//import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Vars;
import frc.robot.subsystems.ArmSubsystem;

public class ArmDegree extends Command {
  /** Creates a new armDegree. */
  double m_kP, m_armDegree;
  ArmSubsystem m_arm;
  public ArmDegree(ArmSubsystem arm, double armDegree) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_arm = arm;
    addRequirements(arm);
    m_armDegree = armDegree;
    
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_arm.setArmAngleBounded(m_armDegree);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return false;
    return Math.abs(m_arm.getArmAngle() - m_armDegree) < Vars.ARM_TOLERANCE;
  }
}
