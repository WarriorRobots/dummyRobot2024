// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.drive;

// import java.util.function.DoubleSupplier;

// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.DrivetrainSubsystem;

// public class ChassisSwerve extends Command {
//   private DoubleSupplier m_speed, m_rotation;
//   private DrivetrainSubsystem m_drive;
//   /** Creates a new TankDrive. */
//   public ChassisSwerve(DoubleSupplier speed, DoubleSupplier rotation, DrivetrainSubsystem drive) {
//     addRequirements(drive);
//     m_drive = drive;
//     m_speed = speed;
//     m_rotation = rotation;
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
    
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     // Make it field
//     ChassisSpeeds chassisSpeeds;
//     chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(m_speed.getAsDouble(),0,m_rotation.getAsDouble(), m_drive.getRotation2d());
//     m_drive.setChassisSpeeds(chassisSpeeds);
  
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
