// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

//import java.util.function.DoubleSupplier;

//import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

//import frc.robot.Vars;
import frc.robot.KnightsSwerve.DriveManipulation;
import frc.robot.commands.drive.SwerveDriveAuto;
//import frc.robot.commands.drive.TankDrive;
import frc.robot.commands.feed.FeedNote;
//import frc.robot.commands.feed.RunFeed;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.shooter.ShooterFeed;
import frc.robot.commands.shooter.ShooterPrep;
import frc.robot.commands.shooter.ShooterRPM;
//import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.FeedSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
//import frc.robot.subsystems.TankSubsystem;

public class AutoMobilityLong extends SequentialCommandGroup {
  /** Creates a new ManualAuto. */
  public AutoMobilityLong(DriveManipulation drive, IntakeSubsystem intake, FeedSubsystem feed, ShooterSubsystem shooter) {
    // Use addRequirements() here to declare subsystem dependencies.

    addCommands(
       new ParallelDeadlineGroup(new WaitCommand(2), new SequentialCommandGroup(
        new ParallelDeadlineGroup(new ShooterPrep(feed), new ShooterRPM(shooter, 4800)),
        new ShooterFeed(shooter, feed, 4800)
      )),
      new ParallelDeadlineGroup(new WaitCommand(4.75), new ParallelCommandGroup(
        new SwerveDriveAuto(drive, .2, 0, 0)
        )
      ),
      new ParallelDeadlineGroup(new WaitCommand(2), new ParallelCommandGroup(
        new SwerveDriveAuto(drive, 0, -.2, 0),
        new ParallelCommandGroup(new RunIntake(intake, ()->.75), new FeedNote(feed, .75))
        )
      )
    );

  }
}
