// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
//import frc.robot.util.triggers.ThresholdTrigger;

/** Handles all Joystick/Xbox input. */
public class IO {
    
  // Verbosity is now handled in Dashboard.java
  
  // private static final Joystick m_leftJoystick = new Joystick(1);
  // private static final Joystick m_rightJoystick = new Joystick(0);
  private static final XboxController m_xbox1 = new XboxController(0);
  private static final XboxController m_xbox2 = new XboxController(1);

  // protected static final JoystickButton leftJoystick_1 = new JoystickButton(m_leftJoystick, 1);
  // protected static final JoystickButton leftJoystick_2 = new JoystickButton(m_leftJoystick, 2);
  // protected static final JoystickButton leftJoystick_3 = new JoystickButton(m_leftJoystick, 3);
  // protected static final JoystickButton leftJoystick_4 = new JoystickButton(m_leftJoystick, 4);
  // protected static final JoystickButton leftJoystick_5 = new JoystickButton(m_leftJoystick, 5);
  // protected static final JoystickButton leftJoystick_6 = new JoystickButton(m_leftJoystick, 6);
  // protected static final JoystickButton leftJoystick_7 = new JoystickButton(m_leftJoystick, 7);
  // protected static final JoystickButton leftJoystick_8 = new JoystickButton(m_leftJoystick, 8);
  // protected static final JoystickButton leftJoystick_9 = new JoystickButton(m_leftJoystick, 9);
  // protected static final JoystickButton leftJoystick_10 = new JoystickButton(m_leftJoystick, 10);
  // protected static final JoystickButton leftJoystick_11 = new JoystickButton(m_leftJoystick, 11);
  // protected static final JoystickButton leftJoystick_12 = new JoystickButton(m_leftJoystick, 12);

  // protected static final JoystickButton rightJoystick_1 = new JoystickButton(m_rightJoystick, 1);
  // protected static final JoystickButton rightJoystick_2 = new JoystickButton(m_rightJoystick, 2);
  // protected static final JoystickButton rightJoystick_3 = new JoystickButton(m_rightJoystick, 3);
  // protected static final JoystickButton rightJoystick_4 = new JoystickButton(m_rightJoystick, 4);
  // protected static final JoystickButton rightJoystick_5 = new JoystickButton(m_rightJoystick, 5);
  // protected static final JoystickButton rightJoystick_6 = new JoystickButton(m_rightJoystick, 6);
  // protected static final JoystickButton rightJoystick_7 = new JoystickButton(m_rightJoystick, 7);
  // protected static final JoystickButton rightJoystick_8 = new JoystickButton(m_rightJoystick, 8);
  // protected static final JoystickButton rightJoystick_9 = new JoystickButton(m_rightJoystick, 9);
  // protected static final JoystickButton rightJoystick_10 = new JoystickButton(m_rightJoystick, 10);
  // protected static final JoystickButton rightJoystick_11 = new JoystickButton(m_rightJoystick, 11);
  // protected static final JoystickButton rightJoystick_12 = new JoystickButton(m_rightJoystick, 12);

  protected static final JoystickButton xbox1_A = new JoystickButton(m_xbox1, 1);
  protected static final JoystickButton xbox1_B = new JoystickButton(m_xbox1, 2);
  protected static final JoystickButton xbox1_X = new JoystickButton(m_xbox1, 3);
  protected static final JoystickButton xbox1_Y = new JoystickButton(m_xbox1, 4);
  protected static final JoystickButton xbox1_LB = new JoystickButton(m_xbox1, 5);
  protected static final JoystickButton xbox1_RB = new JoystickButton(m_xbox1, 6);
  protected static final JoystickButton xbox1_SELECT = new JoystickButton(m_xbox1, 7);
  protected static final JoystickButton xbox1_START = new JoystickButton(m_xbox1, 8);
  protected static final JoystickButton xbox1_L_JOYSTICK = new JoystickButton(m_xbox1, 9);
  protected static final JoystickButton xbox1_R_JOYSTICK = new JoystickButton(m_xbox1, 10);

  protected static final POVButton xbox1_Up = new POVButton(m_xbox1,0);
  protected static final POVButton xbox1_Down = new POVButton(m_xbox1, 180);
  protected static final POVButton xbox1_Left = new POVButton(m_xbox1, 270);
  protected static final POVButton xbox1_Right = new POVButton(m_xbox1, 90);

  protected static final JoystickButton xbox2_A = new JoystickButton(m_xbox2, 1);
  protected static final JoystickButton xbox2_B= new JoystickButton(m_xbox2, 2);
  protected static final JoystickButton xbox2_X = new JoystickButton(m_xbox2, 3);
  protected static final JoystickButton xbox2_Y = new JoystickButton(m_xbox2, 4);
  protected static final JoystickButton xbox2_LB = new JoystickButton(m_xbox2, 5);
  protected static final JoystickButton xbox2_RB = new JoystickButton(m_xbox2, 6);
  protected static final JoystickButton xbox2_SELECT = new JoystickButton(m_xbox2, 7);
  protected static final JoystickButton xbox2_START = new JoystickButton(m_xbox2, 8);
  protected static final JoystickButton xbox2_L_JOYSTICK = new JoystickButton(m_xbox2, 9);
  protected static final JoystickButton xbox2_R_JOYSTICK = new JoystickButton(m_xbox2, 10);

  protected static final POVButton xbox2_Up = new POVButton(m_xbox2,0);
  protected static final POVButton xbox2_Down = new POVButton(m_xbox2, 180);
  protected static final POVButton xbox2_Left = new POVButton(m_xbox2, 270);
  protected static final POVButton xbox2_Right = new POVButton(m_xbox2, 90);
   


  // protected static double getLeftX() {
  //   return m_leftJoystick.getX();
  // }
  
  // protected static double getLeftY() {
  //   return m_leftJoystick.getY() * -1; // * -1 because up is -1 on the joystick
  // }
  
  // protected static double getLeftZ() {
  //   return m_leftJoystick.getZ();
  // }
  
  // protected static double getLeftSlider()  {
  //   return m_leftJoystick.getRawAxis(3); //The third axis is slider
  // }
  
  // protected static double getRightX() {
  //   return m_rightJoystick.getX();
  // }
  
  // protected static double getRightY() {
  //   return m_rightJoystick.getY() * -1; // * -1 because up is -1 on the joystick
  // }
  
  // protected static double getRightZ() {
  //   return m_rightJoystick.getZ();
  // }

  // protected static double getRightSlider()  {
  //   return m_rightJoystick.getRawAxis(3); //The third axis is the slider
  // } 
  
  protected static double getXBox1LeftX()  {
    if(-.08<=m_xbox1.getLeftX() && m_xbox1.getLeftX()<=.08){ //deadzone threshold
      return 0;
    }else{
    return m_xbox1.getLeftX();
    }
  }

  protected static double getXBox1LeftY()
  {
    //up is -1 on joystick
    if(-.08<=m_xbox1.getLeftY()*-1 && m_xbox1.getLeftY()*-1<=.08){ //deadzone threshold
      return 0;
    }else{
    return m_xbox1.getLeftY() * -1;
    }
  }

  protected static double getXBox1RightX() {
    if(-.12<=m_xbox1.getRightX() && m_xbox1.getRightX()<=.12){ //deadzone threshold
      return 0;
    }else{
    return m_xbox1.getRightX();
    }
  }

  protected static double getXBox1RightY() {
    //up is -1 on joystick
    if(-.12<=m_xbox1.getRightY()*-1 && m_xbox1.getRightY()*-1<=.12){ //deadzone threshold
      return 0;
    }else{
    return m_xbox1.getRightY();
    }
  }
  
  protected static double getXBox1LeftTrigger()  {
    return m_xbox1.getLeftTriggerAxis();
  }
  
  protected static double getXBox1RightTrigger() {
    return m_xbox1.getRightTriggerAxis();
  }
}