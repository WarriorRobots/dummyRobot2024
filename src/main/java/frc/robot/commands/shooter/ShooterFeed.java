/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Vars;
// import frc.robot.subsystems.FeedSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterFeed extends Command {

  ShooterSubsystem m_shooter;
  // FeedSubsystem m_feed;
  
  /**
   * A command that runs the shooter and then when the shooter is up to speed, feeds the shooter.
   */
  public ShooterFeed(ShooterSubsystem shooter/*, FeedSubsystem feed*/) {
    m_shooter = shooter;
    addRequirements(m_shooter);
    // m_feed = feed;
    // addRequirements(m_feed);
}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (Math.abs(m_shooter.getRPM()-m_shooter.getCommandedRPM()) < Vars.SHOOTER_TOLERANCE) {
    //   // if the shooter is fast enough, feed it
    //   .feedAtPercent(Vars.SHOOTER_FEED);
    // } else {
    //   // if the shooter is not fast enough...
    //   if (!m_feed.containsBall()) {
    //     // slowly feed it (so a ball is ready to be shot)...
    //     m_feed.feedAtPercent(Vars.SHOOTER_SLOW_FEED);
    //   } else {
    //     // until a ball is ready to be shot
    //     m_feed.stop();
    //   }
    // }
    // run/rev the shooter
    m_shooter.setRPM(m_shooter.getCommandedRPM());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stop();
    // m_feed.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
