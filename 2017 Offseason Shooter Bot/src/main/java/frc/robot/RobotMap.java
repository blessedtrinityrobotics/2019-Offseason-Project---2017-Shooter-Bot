/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  
  // Motor Ports
    // Left Drive Train GB Motors
    public static final int leftMasterMotorPort = 2;
    public static final int  leftSlaveMotor1Port = 3;
    public static final int  leftSlaveMotor2Port = 4;
    // Right Drive Train GB Motors
    public static final int  rightMasterMotorPort = 5;
    public static final int  rightSlaveMotor1Port = 6;
    public static final int  rightSlaveMotor2Port = 7;
    // Intake Motors
    public static final int  leftIntakeMotorPort = 8;
    public static final int  rightIntakeMotorPort = 9;
    // Hopper Motors
    public static final int   frontHopperMotorPort = 10;
    public static final int   backHopperMotorPort = 11;
    // Shooter Motors
    public static final int   shooterMasterMotorPort = 12;
    public static final int   shooterSlaveMotorPort = 13;
  //End of Motor Ports

  // XboxController Ports
    public static final int driveControllerPort = 0;
    public static final int operatorControllerPort = 1;
  //End of Joystick Ports

  //Constants
    public static final int wheelDiameter = 4;
    public static final int unitsPerRotation = 4096;
  //End of Constants

  //Axis
  public static final int leftStickY = 1;
  public static final int rightStickY = 5;
  //End of Axis

  //Buttons
  public static final int xButton = 3;
  //End of Buttons

	
}
