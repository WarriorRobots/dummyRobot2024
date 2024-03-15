// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.arm.ArmDegree;
import frc.robot.commands.drive.ModuleToDegree;
import frc.robot.commands.drive.SwerveDriveCommand;
import frc.robot.commands.drive.SwerveDrivePercent;
import frc.robot.commands.feed.FeedNote;
import frc.robot.commands.feed.RunFeed;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.shooter.ShooterFeed;
import frc.robot.commands.shooter.ShooterPrep;
import frc.robot.commands.shooter.ShooterRPM;
import frc.robot.commands.shooter.ShooterSequence;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.FeedSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
  public static final FeedSubsystem m_feed = new FeedSubsystem();
  public static final ShooterSubsystem m_shooter = new ShooterSubsystem();

  // Drive Command
  // In terms of the robot, the cartesian plane is rotated 90 degrees, i.e X is forward (Y input for controller), Y is horizontal (X input for controller)
  private final SwerveDriveCommand swerveDrive = new SwerveDriveCommand(m_drive, ()->IO.getXBox1LeftX(), ()->IO.getXBox1LeftY(), ()->IO.getXBox1RightX(), ()->DrivetrainSubsystem.FOC);
  //private final SwerveDrivePercent swerveTurn = new SwerveDrivePercent(m_drive, ()->IO.getXBox1RightX(), ()->IO.getXBox1LeftX());
  private final InstantCommand toggleFOC = new InstantCommand(()-> DrivetrainSubsystem.FOC = DrivetrainSubsystem.FOC ? DrivetrainSubsystem.FOC = false : true );
  private final InstantCommand toggleTurboOn = new InstantCommand(()-> Vars.SWERVE_MAX_VELOCITY = .5);
  private final InstantCommand toggleTurboOff = new InstantCommand(()-> Vars.SWERVE_MAX_VELOCITY = 12);
  private final InstantCommand resetGyro = new InstantCommand(()-> m_drive.resetHeading());
  private final InstantCommand resetEncoders = new InstantCommand(()->m_drive.resetEncoders());
  private final ModuleToDegree moveToDegree = new ModuleToDegree(m_drive);

  // Arm Commands
  //TODO update values
  private final ArmDegree m_rest = new ArmDegree(m_arm, 0);
  private final ArmDegree m_scoreNear = new ArmDegree(m_arm, 45);
  private final ArmDegree m_scoreFar = new ArmDegree(m_arm, 0);
  private final ArmDegree m_scoreAmp = new ArmDegree(m_arm, 105);

  // Intake Commands
  private final RunIntake m_forwardIntake = new RunIntake(m_intake, ()->Vars.INTAKE_FORWARD);
  private final RunIntake m_reverseIntake = new RunIntake(m_intake, ()-> Vars.INTAKE_BACKWARD);

  private final ParallelCommandGroup m_pickupForward = new ParallelCommandGroup(
  m_forwardIntake, new FeedNote(m_feed, Vars.FEED_FORWARD)
  );

  private final ParallelCommandGroup m_pickupReverse = new ParallelCommandGroup(
  m_reverseIntake, new RunFeed(m_feed, Vars.FEED_BACKWARD)
  );

  // Shooter Commands
  private final ShooterRPM m_shooterRPM = new ShooterRPM(m_shooter);
  private final ShooterPrep m_shooterPrep = new ShooterPrep(m_feed);
  private final ShooterFeed m_shooterFeed = new ShooterFeed(m_shooter, m_feed);
  private final SequentialCommandGroup m_shooterSequence = new SequentialCommandGroup(
    m_shooterPrep,
    m_shooterFeed
  );
  //private final ShooterFeed m_shooterSequence = new ShooterFeed(m_shooter, m_feed);


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
    // IO.xbox1_RB.whileTrue(toggleTurboOn);
    // IO.xbox1_RB.whileFalse(toggleTurboOff);

    IO.xbox1_Y.toggleOnTrue(toggleFOC);
    IO.xbox1_X.onTrue(resetGyro);

    IO.xbox1_LB.whileTrue(m_pickupForward);
    IO.xbox1_Down.whileTrue(m_pickupReverse);
    IO.xbox1_RB.whileTrue(m_shooterSequence);

    IO.xbox1_Up.whileTrue(m_rest);
    IO.xbox1_A.whileTrue(m_scoreAmp);

    // if(IO.getXBox1LeftTrigger()>=.5){
    //   new ArmDegree(m_arm, 105);
    // }
    // else {
    //   new ArmDegree(m_arm, 0);
    // }    

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
