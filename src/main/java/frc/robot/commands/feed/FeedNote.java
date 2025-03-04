/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.feed;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeedSubsystem;

public class FeedNote extends Command {
  FeedSubsystem m_feed;
  double m_percent;
  /**
   * Run feed at some desired percent.
   */
  public FeedNote(FeedSubsystem feed, double percent) {
    m_feed = feed;
    addRequirements(m_feed);
    m_percent = percent;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_feed.containsNote()==false){
    m_feed.feedAtPercent(m_percent);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_feed.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
