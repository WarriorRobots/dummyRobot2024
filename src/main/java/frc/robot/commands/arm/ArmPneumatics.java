/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Vars;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.Arm;

public class ArmPneumatics extends CommandBase {

  ArmSubsystem m_arm;
  Arm value;
  int count;

  /**
   * A command to set the arm.
   * @see ClimbSubsystem#setArm
   */
  public ArmPneumatics(ArmSubsystem arm, Arm state) {
    m_arm = arm;
    addRequirements(m_arm);
    this.value = state;
  }
  
  @Override
  public void initialize() {
    count = 0;
  }
  @Override
  public void execute() {
    m_arm.setArm(value);
    count++;
  }

  @Override
  public void end(boolean interrupted) {
    m_arm.stopArm();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return count > Vars.PNEUMATIC_LOOP_COUNT;
  }
}
