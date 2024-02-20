// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.Vars;

public class DrivetrainSubsystem extends SubsystemBase {
  // Swerve Module Declaration
  private final SwerveModule frontLeft,frontRight,rearLeft, rearRight;
  // Swerve Drive (entire drivetrain) odometry
  private final SwerveDriveOdometry odometry;
  // *Testing Pose Estimator that uses camera as a input similar to regular Odometry (it is public in order for use in camera subsystem)
  public static SwerveDrivePoseEstimator poseEstimator;
  // Gyro
  private final AHRS navx;
  private final Field2d field;
  //Field Oriented Control (defaulted as true)
  public static boolean FOC = true;

  /** Creates a new DrivetrainSubsystem. */
  public DrivetrainSubsystem() {
    // All Swerve Modules (DriveID, TURNID, DRIVEMOTORPHASE,TURNMOTORPHASE,DRIVEENCODERPHASE, TURN ENCODERPHASE) 
    frontLeft = new SwerveModule(RobotMap.ID_FRONTLEFT_DRIVE, RobotMap.ID_FRONTLEFT_TURN,
    Vars.FRONTLEFT_DRIVE_MOTOR_REVERSED, Vars.FRONTLEFT_TURN_MOTOR_REVERSED, Vars.FRONTLEFT_DRIVE_ENCODER_REVERSED, Vars.FRONTLEFT_TURN_ENCODER_REVERSED,
    RobotMap.ID_FRONTLEFT_CANCODER, Vars.FRONTLEFT_CANCODER_REVERSED, Vars.absoluteEncoderOffsets[0]);

    frontRight = new SwerveModule(RobotMap.ID_FRONTRIGHT_DRIVE, RobotMap.ID_FRONTRIGHT_TURN, 
    Vars.FRONTRIGHT_DRIVE_MOTOR_REVERSED, Vars.FRONTRIGHT_TURN_MOTOR_REVERSED, Vars.FRONTRIGHT_DRIVE_ENCODER_REVERSED, Vars.FRONTRIGHT_TURN_ENCODER_REVERSED,
    RobotMap.ID_FRONTRIGHT_CANCODER, Vars.FRONTRIGHT_CANCODER_REVERSED, Vars.absoluteEncoderOffsets[1]);

    rearLeft = new SwerveModule(RobotMap.ID_REARLEFT_DRIVE, RobotMap.ID_REARLEFT_TURN,
    Vars.REARLEFT_DRIVE_MOTOR_REVERSED,   Vars.REARLEFT_TURN_MOTOR_REVERSED, Vars.REARLEFT_DRIVE_ENCODER_REVERSED, Vars.REARLEFT_TURN_ENCODER_REVERSED,
    RobotMap.ID_REARLEFT_CANCODER, Vars.REARLEFT_CANCODER_REVERSED, Vars.absoluteEncoderOffsets[2]);

    rearRight = new SwerveModule(RobotMap.ID_REARRIGHT_DRIVE, RobotMap.ID_REARRIGHT_TURN, 
    Vars.REARRIGHT_DRIVE_MOTOR_REVERSED, Vars.REARRIGHT_TURN_MOTOR_REVERSED, Vars.REARRIGHT_DRIVE_ENCODER_REVERSED, Vars.REARRIGHT_TURN_ENCODER_REVERSED,
    RobotMap.ID_REARRIGHT_CANCODER,  Vars.REARRIGHT_CANCODER_REVERSED, Vars.absoluteEncoderOffsets[3]);

    // no try/catch on navx because if there is no navx, the auto will break regardless
    navx = new AHRS(I2C.Port.kMXP);
    odometry = new SwerveDriveOdometry(Vars.KINEMATICS, Rotation2d.fromDegrees(getAngle()), getSwervePositions());
    poseEstimator = new SwerveDrivePoseEstimator(Vars.KINEMATICS, getRotation2d(), getSwervePositions(), new Pose2d());
    // shuffleboard field widget to visualize auto routines (publishes a image of the field on smartdashboard)
    field = new Field2d();
    SmartDashboard.putData("field",field);
    //resetHeading();
    //resetEncoders();
    Timer.delay(1);
    setModulesToAbsolute();

    AutoBuilder.configureHolonomic(
      //'this::method' is the same thing as '()->method()' but more concise (I think)
      this::getPose, 
      this::resetPose, 
      this::getSpeeds, 
      this::driveRobotRelative, 
      new HolonomicPathFollowerConfig(
        new PIDConstants(5.0, 0.0, 0.0),
        new PIDConstants(5.0, 0.0, 0.0),
        4.5, 
        0.4, 
        new ReplanningConfig() // Default path replanning config
      ),
      () -> {
      // Boolean supplier that controls when the path will be mirrored for the red alliance
      // This will flip the path being followed to the red side of the field.
      // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
      var alliance = DriverStation.getAlliance();
      if (alliance.isPresent()) {
        return alliance.get() == DriverStation.Alliance.Red;
      }
      return false;
      },
      this
      );

  }
  /**
   * Sets the module to a specific degree (for testing PID)
   * @param degrees
   */
  public void setModuleToDegree(double degrees){
    frontLeft.setTurnDegrees(degrees);
    frontRight.setTurnDegrees(degrees);
    rearLeft.setTurnDegrees(degrees);
    rearRight.setTurnDegrees(degrees);
  }

  // /**
  //  * Sets a turn and drive percent to each module.
  //  * @param turnPercent
  //  * @param drivePercent
  //  */
  public void setDrivePercent(double turnPercent, double drivePercent){
    frontLeft.setDrivePercent(drivePercent);
    frontLeft.setTurnPercent(turnPercent);
    frontRight.setDrivePercent(drivePercent);
    frontRight.setTurnPercent(turnPercent);
    rearLeft.setDrivePercent(drivePercent);
    rearLeft.setTurnPercent(turnPercent);
    rearRight.setDrivePercent(drivePercent);
    rearRight.setTurnPercent(turnPercent);
  }

  /**
   * Sets each module given a desired state.
   * @param state for the modules to be in.
   */
  public void setModuleStates(SwerveModuleState[] state){
    SwerveDriveKinematics.desaturateWheelSpeeds(state,Vars.SWERVE_MAX_VELOCITY);
    frontLeft.setSwerveState(state[0]);
    frontRight.setSwerveState(state[1]);
    rearLeft.setSwerveState(state[2]);
    rearRight.setSwerveState(state[3]);
  }

  /**
   * Sets the state of each module given a chassisspeeds. 
   * @param chassisSpeeds desired chassisSpeeds
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds){
    SwerveModuleState[] state = Vars.KINEMATICS.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(state,Vars.SWERVE_MAX_VELOCITY);
    frontLeft.setSwerveState(state[0]);
    //System.out.println("FL current: " + frontLeft.getTurnDegrees() + "FL Goal" + state[0].angle.getDegrees());
    frontRight.setSwerveState(state[1]);
    //System.out.println("FR current: " + frontRight.getTurnDegrees() + "FR Goal" + state[1].angle.getDegrees());
    rearLeft.setSwerveState(state[2]);
    //System.out.println("RL current: " + rearLeft.getTurnDegrees() + "RL Goal" + state[2].angle.getDegrees());
    rearRight.setSwerveState(state[3]);
    //System.out.println("RR current: " + rearRight.getTurnDegrees() + "RR Goal" + state[3].angle.getDegrees());  
  }

  /**
   * Reset the turn Motors to absolute angle.
   * @param angles
   */
  public void setModulesToAbsolute(){
    frontLeft.setModuleToOffset();
    frontRight.setModuleToOffset();
    rearLeft.setModuleToOffset();
    rearRight.setModuleToOffset();
  }

  /**
   * Retrives the positions of each module state as an array. Order Matters! Should be the same as you set the module states.
   * @return The Positions of each module in an array.
   */
  public SwerveModulePosition[] getSwervePositions(){
   return new SwerveModulePosition[] {frontLeft.getSwerveModulePosition(),frontRight.getSwerveModulePosition(),rearLeft.getSwerveModulePosition(),rearRight.getSwerveModulePosition()};
  }

  /**
   *  SwerveState array of all modules
   * @return velocity and angle of module.
   */
  public SwerveModuleState[] getSwerveStates(){
    return new SwerveModuleState[] {frontLeft.getSwerveState(), frontRight.getSwerveState(),rearLeft.getSwerveState(), rearRight.getSwerveState()};
  }

  /**
   * Retrives the absolute position of each module.
   * @return an array with each absolute position if each module.
   */
  // public double[] getAbsolutePositions(){
  //   return new double[] {frontLeft.getAbsolutePosition(), frontRight.getAbsolutePosition(), rearLeft.getAbsolutePosition(), rearRight.getAbsolutePosition()};
  // }

  /**
   * Retrives the absolute position of each module.
   * @return an array with each absolute position if each module.
   */
  public double[] getPositions(){
    return new double[] {frontLeft.getTurnDegrees(), frontRight.getTurnDegrees(), rearLeft.getTurnDegrees(), rearRight.getTurnDegrees()};
  }
 
  /**
   *  Average Distance Travelled in inches.
   * @return the average distance of the robot in inches.
   */
  public double getDistance(){
    return ((frontLeft.getDrivePosition() + frontRight.getDrivePosition() + rearLeft.getDrivePosition() + rearRight.getDrivePosition()) / 4);
  }

  /**
   * Average Velocity of the robot in inches/second. 
   * @return
   */
  public double getVelocity(){
    return ((frontLeft.getDriveVelocity() + frontRight.getDriveVelocity() + rearLeft.getDriveVelocity() + rearRight.getDriveVelocity()) / 4);
  }

  /**
   * Returns the degree value of the Rotation2d.
   * @return The degree value of the Rotation2d. (Angle within (-180, 180)
   */
  public double getHeading(){
    return navx.getRotation2d().getDegrees();
  }

  /**
   * Retrieves the position of the robot on the field.
   * @return The pose of the robot (x and y are in meters).
   */
  public Pose2d getPose(){
    return odometry.getPoseMeters();
  }

  /**
   * Retrives the Rotation 2d of the robot in degrees.
   * @return Rotation2d of the robot.
   */
  public Rotation2d getRotation2d(){
      return navx.getRotation2d();
  }

  public double getRoll(){
    return navx.getRoll();
  }

  /**
   * Retrieves the angle of the navx in degrees. Angle is continuous i.e will go past 360 degrees.
   * @return angle of the robot.
   */
  public double getAngle(){
    return navx.getAngle();
  }
  
  public ChassisSpeeds getSpeeds(){
    return Vars.KINEMATICS.toChassisSpeeds(getSwerveStates());
  }

  public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
    driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPose().getRotation()));
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
    setChassisSpeeds(targetSpeeds);
  } 

  /**
   * Resets the Odometry
   */
  public void resetPose(Pose2d pose){
    odometry.resetPosition(navx.getRotation2d(), getSwervePositions(), pose);
  }

  /**
   * Resets the the drive and turn motor for each module to zero. 
   */
  public void resetEncoders(){
    frontLeft.resetEncoders();
    frontRight.resetEncoders();
    rearLeft.resetEncoders();
    rearRight.resetEncoders();
  }

  /**
   * Resets the heading.
   */
  public void resetHeading(){
    navx.reset();
  }

  /**
   * Stops all modules. 
   */
  public void stopModules(){
    frontLeft.stop();
    frontRight.stop();
    rearLeft.stop();
    rearRight.stop();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(getRotation2d(),getSwervePositions());
    field.setRobotPose(getPose());
    poseEstimator.update(getRotation2d(), getSwervePositions());
    SmartDashboard.putBoolean("FOC", FOC);
    // SmartDashboard.putNumber("AVR Distance ",getDistance());
    // SmartDashboard.putNumber("Avr Velcotiy", getVelocity());
     SmartDashboard.putNumber("Navx Angle", getAngle());
     SmartDashboard.putString("Rotation2d", getRotation2d().toString());
     SmartDashboard.putNumber("Heading", getHeading());
     SmartDashboard.putNumberArray("Absolute Positions", getPositions());
     SmartDashboard.putNumber("Front Left", frontLeft.getTurnDegrees()*18);
     SmartDashboard.putNumber("Front Right", frontRight.getTurnDegrees()*18);
     SmartDashboard.putNumber("Rear Left", rearLeft.getTurnDegrees()*18);
     SmartDashboard.putNumber("Rear RIght", rearRight.getTurnDegrees()*18);
    //SmartDashboard.putNumberArray("Absolute Encoder Positions", getAbsolutePositions());
    // Each state tells me the angle and speed of each module. (for testing/verifcation)
    // SmartDashboard.putString("FrontLeft State", getSwerveStates()[0].toString());
    // SmartDashboard.putString("FrontRight State", getSwerveStates()[1].toString());
    // SmartDashboard.putString("RearLeft State", getSwerveStates()[2].toString());
    // SmartDashboard.putString("RearRight State", getSwerveStates()[3].toString());
    // SmartDashboard.putNumberArray("Positions", getPositions());
  }

}
