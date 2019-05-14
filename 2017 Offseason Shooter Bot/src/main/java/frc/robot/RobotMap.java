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
    public static int leftMasterMotorPort = 2;
    public static int  leftSlaveMotor1Port = 3;
    public static int  leftSlaveMotor2Port = 4;
    // Right Drive Train GB Motors
    public static int  rightMasterMotorPort = 5;
    public static int  rightSlaveMotor1Port = 6;
    public static int  rightSlaveMotor2Port = 7;
    // Intake Motors
    public static int  leftIntakeMotorPort = 8;
    public static int  rightIntakeMotorPort = 9;
    // Hopper Motors
    public static int   frontHopperMotorPort = 10;
    public static int   backHopperMotorPort = 11;
    // Shooter Motors
    public static int   shooterMasterMotorPort = 12;
    public static int   shooterSlaveMotorPort = 13;
  //End of Motor Ports

  // Joysick Ports
    public static int driveJoyPort = 0;
    public static int operatorJoyPort = 1;
  //End of Joystick Ports

  //Constants
    public static int wheelDiameter = 4;
  //End of Constants
  
}
