/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
//import frc.robot.Vars;
import frc.robot.subsystems.FeedSubsystem;
// import frc.robot.subsystems.FeedSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterPrep extends Command {

  ShooterSubsystem m_shooter;
  FeedSubsystem m_feed;
  private double counter = 0;

  /**
   * A command to clear the shooter of any notes (usually before shooting is ran.)
   */
  public ShooterPrep(FeedSubsystem feed) {
    m_feed = feed;
    addRequirements(m_feed);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //m_feed.feedAtPercent(Vars.FEED_BACKWARD);
    counter++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_feed.stop();
    counter = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // make sure a note is not in the shooter
    //return !m_feed.containsNote();
    return counter >= 50;
  }
}
