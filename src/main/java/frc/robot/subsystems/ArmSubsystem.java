/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Arm Subsystem. */
public class ArmSubsystem extends SubsystemBase {

  private DoubleSolenoid m_arm;

  public ArmSubsystem() {
    m_arm = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, RobotMap.ID_ARM_F, RobotMap.ID_ARM_B);
    m_arm.set(DoubleSolenoid.Value.kOff);
  }

  /**
   * Set the value of the arm. <p>
   * @param value A {@link DoubleSolenoid.Value}
   * @see #stopArm
   * @see #engageArm
   * @see #disengageArm
   */
  public void setArm(Arm state) {
    m_arm.set(state.getValue());
  }

  /**
   * kOff -> None/stop <p>
   * kReverse -> Engage arm <p>
   * kForward -> Disengage arm <p>
   */
  public static enum Arm {
    stop(DoubleSolenoid.Value.kOff),
    engage(DoubleSolenoid.Value.kReverse),
    disengage(DoubleSolenoid.Value.kForward);
    public DoubleSolenoid.Value value;
    Arm(DoubleSolenoid.Value value) {
      this.value = value;
    }
    public DoubleSolenoid.Value getValue() {
      return value;
    }
  }

  /**
   * Used to stop giving pressure to the arm in either direction.
   * This neither remove nor adds pressure, therefore maintaining the previous state.
   * This must be called after several loops of the forwards as to not overwork the pneumatics.
   */
  public void stopArm() {
    setArm(Arm.stop);
  }

  /**
   * Pulls arm into the scoring position.
   */
  public void engageArm() {
    setArm(Arm.engage);
  }

  /**
   * Pushes arm out of the scoring position.
   */
  public void disengageArm() {
    setArm(Arm.disengage);
  }

  /**
   * The stop command is private because the two components of the climb should be stopped
   * individually as there is no time either should be stopped in unison.
   */
  @SuppressWarnings("unused")
  private void stop() {}

  @Override
  public void periodic() {
    //putDashboard();
  }

  // /**
  //  * Puts information about this subsystem on the dashboard.
  //  */
  // public void putDashboard() {
  //   switch (DashboardContainer.getInstance().getVerbosity()) {
  //     case 2:
  //       SmartDashboard.putString("Arm", m_arm.get().toString());
  //     case 1:
	// 		  break;
	// 		default:
  //       break;
  //   }
  // }

}
