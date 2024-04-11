// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.drive;


// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Vars;
// import frc.robot.subsystems.DrivetrainSubsystem;

// public class ModuleToDegree extends Command {
//   /** Creates a new moduleToDegree. */
//   DrivetrainSubsystem m_drive;
//   public ModuleToDegree(DrivetrainSubsystem drive) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     m_drive = drive;
//     addRequirements(drive);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     m_drive.setModuleToDegree(90);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     m_drive.stopModules();
//   }

//   int counter = 0;

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return Math.abs(m_drive.getSwerveStates()[0].angle.getDegrees() - 90) <= Vars.SWERVE_ANGLE_TOLERANCE && Math.abs(m_drive.getSwerveStates()[1].angle.getDegrees() - 90) <= Vars.SWERVE_ANGLE_TOLERANCE && Math.abs(m_drive.getSwerveStates()[2].angle.getDegrees() - 90) <= Vars.SWERVE_ANGLE_TOLERANCE && Math.abs(m_drive.getSwerveStates()[3].angle.getDegrees() - 90) <= Vars.SWERVE_ANGLE_TOLERANCE;
//   }
// }
