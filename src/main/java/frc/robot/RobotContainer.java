// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.arm.ArmPneumatics;
import frc.robot.commands.drive.ModuleToDegree;
import frc.robot.commands.drive.SwerveDriveCommand;
import frc.robot.commands.drive.SwerveDrivePercent;
import frc.robot.commands.intake.RunIntake;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ArmSubsystem.Arm;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static final DrivetrainSubsystem m_drive = new DrivetrainSubsystem();
  public static final ArmSubsystem m_arm = new ArmSubsystem();
  public static final IntakeSubsystem m_intake = new IntakeSubsystem();

  // Drive Command
  // In terms of the robot, the cartesian plane is rotated 90 degrees, i.e X is forward (Y input for controller), Y is horizontal (X input for controller)
  private final SwerveDriveCommand swerveDrive = new SwerveDriveCommand(m_drive, ()->IO.getXBox1LeftX(), ()->IO.getXBox1LeftY(), ()->IO.getXBox1RightX(), ()->DrivetrainSubsystem.FOC);
  private final InstantCommand toggleFOC = new InstantCommand(()-> DrivetrainSubsystem.FOC = DrivetrainSubsystem.FOC ? DrivetrainSubsystem.FOC = false : true );
  private final InstantCommand toggleTurboOn = new InstantCommand(()-> Vars.SWERVE_MAX_VELOCITY = .5);
  private final InstantCommand toggleTurboOff = new InstantCommand(()-> Vars.SWERVE_MAX_VELOCITY = 12);
  private final InstantCommand resetGyro = new InstantCommand(()-> m_drive.resetHeading());
  private final InstantCommand resetEncoders = new InstantCommand(()->m_drive.resetEncoders());
  private final ModuleToDegree moveToDegree = new ModuleToDegree(m_drive);

  // Arm Commands
  private final ArmPneumatics m_disengageArm = new ArmPneumatics(m_arm, Arm.engage);
  private final ArmPneumatics m_engageArm = new ArmPneumatics(m_arm, Arm.disengage);

  // Intake Commands
  private final RunIntake m_forwardIntake = new RunIntake(m_intake, ()->Vars.INTAKE_FORWARD);
  private final RunIntake m_backwardIntake = new RunIntake(m_intake, ()-> Vars.INTAKE_BACKWARD);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    CommandScheduler.getInstance().setDefaultCommand(m_drive,swerveDrive);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    IO.xbox1_B.toggleOnTrue(toggleFOC);
    IO.xbox1_A.onTrue(moveToDegree);
    IO.xbox1_X.onTrue(resetGyro);
    IO.xbox1_Y.onTrue(resetEncoders);
    IO.xbox1_RB.whileTrue(toggleTurboOn);
    IO.xbox1_RB.whileFalse(toggleTurboOff);

    IO.xbox2_RB.onTrue(m_engageArm);
    IO.xbox2_LB.onTrue(m_disengageArm);
    IO.xbox2_A.whileTrue(m_forwardIntake);
    IO.xbox2_B.whileTrue(m_backwardIntake);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return AutoContainer.getInstance().getAutoCommand();
  }

}
