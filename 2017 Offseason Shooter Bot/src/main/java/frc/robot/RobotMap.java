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
    public static int leftMasterMotor = 2;
    public static int  leftSlaveMotor1 = 3;
    public static int  leftSlaveMotor2 = 4;
    // Right Drive Train GB Motors
    public static int  rightMasterMotor = 5;
    public static int  rightSlaveMotor1 = 6;
    public static int  rightSlaveMotor2 = 7;
    // Intake Motors
    public static int  leftIntakeMotor = 8;
    public static int  rightIntakeMotor = 9;
    // Hopper Motors
    public static int   frontHopperMotor = 10;
    public static int   backHopperMotor = 11;
    // Shooter Motors
    public static int   shooterMasterMotor = 12;
    public static int   shooterSlaveMotor = 13;
  //End of Motor Ports

  // Joysick Ports
    public static int driveJoyPort = 0;
    public static int operatorJoyPort = 1;
  //End of Joystick Ports

  //Constants
    public static int wheelDiameter = 4;
  //End of Constants
  
}
