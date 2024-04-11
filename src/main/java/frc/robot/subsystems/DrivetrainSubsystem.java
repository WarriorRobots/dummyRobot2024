package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkFlex;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.Vars;
import frc.robot.KnightsSwerve.TurnMotor;

public class DrivetrainSubsystem {
    private double[] swerveModuleAngles = {0, 0, 0, 0}; // in radians
    private double[] swerveModuleSpeeds = {0, 0, 0, 0}; // in meters per second

    boolean brakeRotation = false;

    private double previousState;
    private double setState;
    private double errorInState;
    private double[] pidStuff = {.3, 0, .1};

    public static double[] swerveAngleOffset = {
        -230 * (Math.PI /180),
        25 * (Math.PI /180),
        5 * (Math.PI /180),
        200 * (Math.PI /180),
    };

    private double x;
    private double y;
    private double rotation;

    public AHRS navx;

    private final TalonFX frontLeftDrive, frontRightDrive, rearLeftDrive, rearRightDrive;
    private final TalonFX frontLeftTurn, frontRightTurn, rearLeftTurn, rearRightTurn;
    private final CANcoder frontLeftTurnEncoder, frontRightTurnEncoder, rearLeftTurnEncoder, rearRightTurnEncoder;

    public DrivetrainSubsystem() {

    frontLeftDrive  = new TalonFX(RobotMap.ID_FRONTLEFT_DRIVE);
    frontRightDrive = new TalonFX(RobotMap.ID_FRONTRIGHT_DRIVE);
    rearLeftDrive   = new TalonFX(RobotMap.ID_REARLEFT_DRIVE);
    rearRightDrive  = new TalonFX(RobotMap.ID_REARRIGHT_DRIVE);

    frontLeftTurn = new TalonFX(RobotMap.ID_FRONTLEFT_TURN);
    frontRightTurn = new TalonFX(RobotMap.ID_FRONTRIGHT_TURN);
    rearLeftTurn = new TalonFX(RobotMap.ID_REARLEFT_TURN);
    rearRightTurn = new TalonFX(RobotMap.ID_REARRIGHT_TURN);

    frontLeftTurnEncoder = new CANcoder(RobotMap.ID_FRONTLEFT_CANCODER);
    frontRightTurnEncoder = new CANcoder(RobotMap.ID_FRONTRIGHT_CANCODER);
    rearLeftTurnEncoder = new CANcoder(RobotMap.ID_REARLEFT_CANCODER);
    rearRightTurnEncoder = new CANcoder(RobotMap.ID_REARRIGHT_CANCODER);

    navx = new AHRS(edu.wpi.first.wpilibj.SPI.Port.kMXP);
    
    }

    public void setNewCenterState(double x, double y, double rotation) {
        
        this.x = x;
        this.y = y;
        this.rotation = rotation;

        double oldX = x;
        double oldY = y;

        double angleFromNavX = navx.getAngle() / 180 * Math.PI;
        angleFromNavX += Math.PI + Robot.gyroOffset;
        x = oldX * Math.cos(angleFromNavX) - oldY * Math.sin(angleFromNavX);
        y = oldX * Math.sin(angleFromNavX) + oldY * Math.cos(angleFromNavX);

        if ((x < .1 && x > -.1) && (y < .1 && y > -.1) && (rotation < .1 && rotation > -.1)) {
            x = 0;
            y = 0;
            rotation = 0;
            for (int i = 0; i < 4; i++) {
                swerveModuleSpeeds[i] = 0;
            }
        } else {
            // V_p
            double[] positionVector = {x, y};
            // v_s
            double[][] newRotationVector = new double[4][2];
            double[][] constantRotationVector = new double[4][2];
            
            // we need to get the x and y components of the rotation vector
            for (int i = 0; i < 4; i++) {
                newRotationVector[i][0] = Math.cos(Constants.constantRotationAngle[i]);
                newRotationVector[i][1] = Math.sin(Constants.constantRotationAngle[i]);
            }
            
            // we then need to scale the rotation vector by the rotation value
            for (int i = 0; i < 4; i++) {
                newRotationVector[i][0] *= rotation;
                newRotationVector[i][1] *= rotation;
            }
            
            // now that we have the two scaled and set vector arrays, we can add them together
            
            // V_f
            double[][] finalVector = new double[4][2];
            for (int i = 0; i < 4; i++) {
                finalVector[i][0] = positionVector[0] + newRotationVector[i][0];
                finalVector[i][1] = positionVector[1] + newRotationVector[i][1];
            }
            
            if (rotation < .1 && rotation > -.1) {
                brakeRotation = false;
            } else {
                if (Math.sqrt(positionVector[0] * positionVector[0] + positionVector[1] * positionVector[1]) < .1) {
                    brakeRotation = true;
                } else {
                    brakeRotation = false;
                }
            }

            // calculate the hypotenuse
            // double hypotenuse = Math.sqrt(finalVector[0][0] * finalVector[0][0] + finalVector[0][1] * finalVector[0][1]);
            double[] hypotenuse = new double[4];

            for (int i = 0; i<4; ++i) {
                hypotenuse[i] = Math.sqrt(finalVector[i][0] * finalVector[i][0] + finalVector[i][1] * finalVector[i][1]);
            }
            // calculate the angle using the previously commented out for loop, use hypotenuse
            for (int i = 0; i < 4; i++) {
                if (finalVector[i][1] > 0) {
                    swerveModuleAngles[i] = Math.acos(finalVector[i][0] / hypotenuse[i]);
                } else if (finalVector[i][1] < 0) {
                    swerveModuleAngles[i] = Math.PI * 2 - Math.acos(finalVector[i][0] / hypotenuse[i]);
                } 
            }

            SmartDashboard.putNumber("Front Left Angle", swerveModuleAngles[0]);
            SmartDashboard.putNumber("Front Right Angle", swerveModuleAngles[1]);
            SmartDashboard.putNumber("Back Left Angle", swerveModuleAngles[2]);
            SmartDashboard.putNumber("Back Right Angle", swerveModuleAngles[3]);

            // dont forget to add the offset
            for (int i = 0; i < 4; i++) {
                swerveModuleAngles[i] += swerveAngleOffset[i]  + (Math.PI /2);
            }

            // now that al the angles are set, we can set the speeds
            for (int i = 0; i < 4; i++) {
                swerveModuleSpeeds[i] = Math.sqrt(finalVector[i][0] * finalVector[i][0] + finalVector[i][1] * finalVector[i][1]);
            }}
    }
    public void setStates(boolean precisionMode) {
        setDesiredAngle(frontLeftTurnEncoder, swerveModuleAngles[0]);
        setDesiredAngle(frontRightTurnEncoder, swerveModuleAngles[1]);
        setDesiredAngle(rearLeftTurnEncoder, swerveModuleAngles[2]);
        setDesiredAngle(rearRightTurnEncoder, swerveModuleAngles[3]);
    
        runToState(frontLeftTurn, frontLeftTurnEncoder);
        runToState(frontRightTurn, frontRightTurnEncoder);
        runToState(rearLeftTurn, rearLeftTurnEncoder);
        runToState(rearRightTurn, rearRightTurnEncoder);
    
        double sum = 0;
        double avg = 0;
        
        // now we can set the speeds
        
        // average the speeds to each of the motors
        for(int i = 0; i < swerveModuleSpeeds.length; i++){  
            //getting elements from the list and adding to the variable sum   
            sum = sum + swerveModuleSpeeds[i];  
            //finds the average of the list  
            avg = sum / swerveModuleSpeeds.length;   

        }
        /* 
        if (brakeRotation) {
            frontLeftDrive.setIdleMode(CANSparkFlex.IdleMode.kBrake);
            frontRightDrive.setIdleMode(CANSparkFlex.IdleMode.kBrake);
            backLeftDrive.setIdleMode(CANSparkFlex.IdleMode.kBrake);
            backRightDrive.setIdleMode(CANSparkFlex.IdleMode.kBrake);
        } else {
            frontLeftDrive.setIdleMode(CANSparkFlex.IdleMode.kCoast);
            frontRightDrive.setIdleMode(CANSparkFlex.IdleMode.kCoast);
            backLeftDrive.setIdleMode(CANSparkFlex.IdleMode.kCoast);
            backRightDrive.setIdleMode(CANSparkFlex.IdleMode.kCoast);
        }


        //Apply power deadband to keep motors from coasting
        if(Math.abs(avg) < .1){
            frontLeftDrive.stopMotor();
            frontRightDrive.stopMotor();
            backLeftDrive.stopMotor();
            backRightDrive.stopMotor();
        }else{
            if(precisionMode){
                frontLeftDrive.set (swerveModuleSpeeds[0] * .2);
                frontRightDrive.set(swerveModuleSpeeds[1] * .2);
                backLeftDrive.set  (swerveModuleSpeeds[2] * .2);
                backRightDrive.set (swerveModuleSpeeds[3] * .2);


            }else{
                frontLeftDrive.set (swerveModuleSpeeds[0] * .7);
                frontRightDrive.set(swerveModuleSpeeds[1] * .7);
                backLeftDrive.set  (swerveModuleSpeeds[2] * .7);
                backRightDrive.set (swerveModuleSpeeds[3] * .7);

                frontLeftDrive.setIdleMode(CANSparkFlex.IdleMode.kCoast);
                frontRightDrive.setIdleMode(CANSparkFlex.IdleMode.kCoast);
                backLeftDrive.setIdleMode(CANSparkFlex.IdleMode.kCoast);
                backRightDrive.setIdleMode(CANSparkFlex.IdleMode.kCoast);

            }

        }
        */

        if (precisionMode) {
            power(.2, swerveModuleSpeeds);
        } else {
            power(.7, swerveModuleSpeeds);
        }

    }

    public void power(double multiplier, double[] speeds) {
        frontLeftDrive.set(multiplier * speeds[0]);
        frontRightDrive.set(multiplier * speeds[1]);
        rearLeftDrive.set(multiplier * speeds[2]);
        rearRightDrive.set(multiplier * speeds[3]);
    }

    public void setDesiredAngle(CANcoder turnEncoder, double radianMeasure) {
    
        errorInState = previousState - turnEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI;
        //absolute the error
        errorInState = Math.abs(errorInState);
    
        previousState = turnEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI;
        setState = radianMeasure;
        while (setState >= Math.PI * 2)
        {
            setState -= Math.PI * 2;
        }
        while (setState < 0)
        {
            setState += Math.PI * 2;
        }
    
    }

    public void runToState(TalonFX turnMotor, CANcoder turnEncoder) {
        //logic is to set the new position as a "0" and then run the motor to that position
        //right now only p
        double power = 0;
        
        double whereItIs = turnEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI;
    
        double[] otherValidStates = {
            setState + (Math.PI *2),
            setState - (Math.PI *2)
        };
        
        // the encoder (whereItIs) resets to 0 every 2pi, but we can tell the power to go
        // beyond 2pi, and it wont matter until it resets to 0
        // so, we just need to set it for the closest congruent angle
    
        double[] differences = {
            whereItIs - setState,
            whereItIs - otherValidStates[0],
            whereItIs - otherValidStates[1]
        };
    
        int smallestDifference = 0;
    
        // find the smallest difference
        if (Math.abs(differences[0]) < Math.abs(differences[1]) && Math.abs(differences[0]) < Math.abs(differences[2])) {
            smallestDifference = 0;
        } else if (Math.abs(differences[1]) < Math.abs(differences[2]) && Math.abs(differences[1]) < Math.abs(differences[0])) {
            smallestDifference = 1;
        } else if (Math.abs(differences[2]) < Math.abs(differences[0]) && Math.abs(differences[2]) < Math.abs(differences[1])) {
            smallestDifference = 2;
        }
    
        // note that the negative power goes forward
        double powerProportion = pidStuff[0] * differences[smallestDifference];
    
        power = powerProportion;
        
        turnMotor.set(power);
    }

    public void stopModules(){
        frontLeftDrive.stopMotor();
        frontRightDrive.stopMotor();
        rearLeftDrive.stopMotor();
        rearRightDrive.stopMotor();

        frontLeftTurn.stopMotor();
        frontRightTurn.stopMotor();
        rearLeftTurn.stopMotor();
        rearRightTurn.stopMotor();
    }

}
