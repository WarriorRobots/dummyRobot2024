// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

//import java.util.function.DoubleSupplier;

//import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

//import frc.robot.Vars;
import frc.robot.KnightsSwerve.DriveManipulation;
//import frc.robot.commands.drive.TankDrive;
//import frc.robot.commands.feed.FeedNote;
//import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.shooter.ShooterFeed;
import frc.robot.commands.shooter.ShooterPrep;
import frc.robot.commands.shooter.ShooterRPM;
import frc.robot.subsystems.FeedSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
//import frc.robot.subsystems.TankSubsystem;

public class ManualAuto extends SequentialCommandGroup {
  /** Creates a new ManualAuto. */
  public ManualAuto(DriveManipulation drive, IntakeSubsystem intake, FeedSubsystem feed, ShooterSubsystem shooter) {
    // Use addRequirements() here to declare subsystem dependencies.

    addCommands(
      new ParallelDeadlineGroup(new WaitCommand(2), new SequentialCommandGroup(
        new ParallelDeadlineGroup(new ShooterPrep(feed), new ShooterRPM(shooter, 4800)),
        new ShooterFeed(shooter, feed, 4800)
      ))
      // new ParallelDeadlineGroup(new WaitCommand(3), new ParallelCommandGroup(
      //   new TankDrive(tank, ()->-.5, ()->-.5)),
      //   new ParallelCommandGroup(new RunIntake(intake, ()->Vars.INTAKE_FORWARD), new FeedNote(feed, Vars.FEED_FORWARD))
      // ),
      // new ParallelDeadlineGroup(new WaitCommand(3), new TankDrive(tank, ()->.5, ()->.5)),
      // new ParallelDeadlineGroup(new WaitCommand(2), new ShooterFeed(shooter, feed))
    );
  }
}
