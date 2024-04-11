// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.drive;

// import java.util.function.BooleanSupplier;
// import java.util.function.DoubleSupplier;

// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.DrivetrainSubsystem;

// public class SwerveDriveCommand extends Command {
//   /** Creates a new SwerveDriveCommand. */
//   private final DrivetrainSubsystem drive;
//   private final DoubleSupplier yInput,xInput,rotInput;
//   private final BooleanSupplier fieldOriented;

//   public SwerveDriveCommand(DrivetrainSubsystem drive, DoubleSupplier yInput, DoubleSupplier xInput, DoubleSupplier rotInput, BooleanSupplier fieldOriented ) {
//     this.drive = drive;
//     addRequirements(drive);
//     this.yInput = yInput;
//     this.xInput = xInput;
//     this.rotInput = rotInput;
//     this.fieldOriented = fieldOriented;
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
    
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//      ChassisSpeeds chassisSpeeds;
//      if(fieldOriented.getAsBoolean()){
//       chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xInput.getAsDouble() * 2,-yInput.getAsDouble() * 8/3,-rotInput.getAsDouble() * 6, drive.getRotation2d());
//     }else{
//       chassisSpeeds = new ChassisSpeeds(xInput.getAsDouble() * 2,-yInput.getAsDouble() * 8/3,-rotInput.getAsDouble() * 6);
//   }
//     System.out.println("chassisSpeeds"+chassisSpeeds);
//     drive.setChassisSpeeds(chassisSpeeds);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     drive.stopModules();
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
