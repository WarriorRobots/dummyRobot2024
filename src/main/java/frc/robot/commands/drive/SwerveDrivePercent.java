// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.drive;

// import java.util.function.DoubleSupplier;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.DrivetrainSubsystem;

// public class SwerveDrivePercent extends Command {
//   /** Creates a new SwerveDrivePercent. */
//   DrivetrainSubsystem m_drive;
//   DoubleSupplier m_turnPercent, m_drivePercent;
//   public SwerveDrivePercent(DrivetrainSubsystem drive,DoubleSupplier turnPercent, DoubleSupplier drivePercent) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     m_drive = drive;
//     addRequirements(drive);
//     m_turnPercent = turnPercent;
//     m_drivePercent = drivePercent;
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     m_drive.setDrivePercent(m_turnPercent.getAsDouble(), m_drivePercent.getAsDouble());
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     m_drive.stopModules();
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
