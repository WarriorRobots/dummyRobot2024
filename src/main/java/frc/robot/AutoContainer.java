// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.DashboardContainer.TabsIndex;
import frc.robot.commands.feed.FeedNote;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.shooter.ShooterFeed;
import frc.robot.commands.shooter.ShooterPrep;
import frc.robot.commands.shooter.ShooterRPM;
import frc.robot.subsystems.FeedSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import java.time.Instant;

import com.pathplanner.lib.auto.AutoBuilder;


/**
 * Singleton to handle auto selection
 */
public class AutoContainer {
  private ShuffleboardTab autoTab = DashboardContainer.getInstance().getTab(TabsIndex.kAuto);
  private SendableChooser<Command> chooser;
  //private SendableChooser<Command> chooser = new SendableChooser<Command>();
  private static AutoContainer instance = null;

  // This constructor is private because it is a singleton
  private AutoContainer() {}

  private void init() {

  // Command Initalization
  // final ShooterPrep m_shooterPrep = new ShooterPrep(new FeedSubsystem());
  // final ShooterFeed m_shooterFeed = new ShooterFeed(new ShooterSubsystem(), new FeedSubsystem());

  // final RunIntake m_forwardIntake = new RunIntake(new IntakeSubsystem(), ()->Vars.INTAKE_FORWARD);
  // final FeedNote m_feedNote = new FeedNote(new FeedSubsystem(), Vars.FEED_FORWARD);

  // Event marker configuration
  NamedCommands.registerCommand("test", new InstantCommand());
  NamedCommands.registerCommand("shootNear", new SequentialCommandGroup(new ShooterPrep(new FeedSubsystem()), new ShooterFeed(new ShooterSubsystem(), new FeedSubsystem())));
  NamedCommands.registerCommand("intake", new ParallelCommandGroup(new RunIntake(new IntakeSubsystem(), ()->Vars.INTAKE_FORWARD), new FeedNote(new FeedSubsystem(), Vars.FEED_FORWARD)));

  // Auto configuration
  chooser = new SendableChooser<Command>();
  //chooser = AutoBuilder.buildAutoChooser();

  chooser.addOption("None", new InstantCommand());

  // chooser.addOption("Test", AutoBuilder.buildAuto("New Auto"));
  chooser.addOption("Test", new PathPlannerAuto("New Auto")); 
  chooser.addOption("Backup", new PathPlannerAuto("Backup"));
  chooser.addOption("Two Note Test", new PathPlannerAuto("Two Note Single Path")); 

  autoTab.add("Auto Selector", chooser).withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(3, 0).withSize(2,1);
}

  /**
   * Gets the AutoContainer instance.
   */
  public static AutoContainer getInstance() {
  if (instance == null) {
    instance = new AutoContainer();
    instance.init();
  }
  return instance;
  }

  public Command getAutoCommand() {
    return chooser.getSelected();
  }
  
}