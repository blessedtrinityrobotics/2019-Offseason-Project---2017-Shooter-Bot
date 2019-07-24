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
    public static final int  leftMasterMotorPort     = 3;
    public static final int  leftSlaveMotor1Port     = 4;
    public static final int  leftSlaveMotor2Port     = 8;
    // Right Drive Train GB Motors
    public static final int  rightMasterMotorPort    = 1;
    public static final int  rightSlaveMotor1Port    = 5;
    public static final int  rightSlaveMotor2Port    = 6;
    // Intake Motors
    public static final int  leftIntakeMotorPort     = 0;
    public static final int  rightIntakeMotorPort    = 1;
    // Hopper Motors
    public static final int  frontHopperMotorPort    = 3;
    public static final int  backHopperMotorPort     = 11;
    // Shooter Motors
    public static final int  shooterMasterMotorPort  = 2;
    public static final int  shooterSlaveMotorPort   = 7;
  //End of Motor Ports

  // XboxController Ports
    public static final int driveControllerPort      = 0;
    public static final int operatorControllerPort   = 1;
  //End of Joystick Ports

  //Constants
    public static final int wheelDiameter   = 4;    // Wheel Diameter
    public static final int PPR             = 4096; // Pulse Per Revolution (SRX Mag Encoder)
    public static final double PPI          = (PPR)/(2*Math.PI*(wheelDiameter/2)); // Pulse Per Inch
    public static final double turningPower = 0.5;  // Turning Power for Drive (%)
    public static final double RPM          = 4000; // Shooter Speed (RPM)
    public static final double HopperSpeed  = 0.75; // Hopper Speed (Volts)
    public static final double IntakeSpeed  = 1.0;  // Intake Speed (Volts)
    public static final double waitTime     = 1.0;  // Time to wait before grabing final gyro angle for vision approach
  //End of Constants

  //Axis
    public static final int leftStickX       = 0;  
    public static final int leftStickY       = 1;
    public static final int leftTriggerAxis  = 2;
    public static final int rightTriggerAxis = 3;
    public static final int rightStickX      = 4;
    public static final int rightStickY      = 5;
  //End of Axis

  //Buttons
    public static final int aButton           = 1;
    public static final int bButton           = 2;
    public static final int xButton           = 3;
    public static final int yButton           = 4;
    public static final int leftBumperButton  = 5;
    public static final int rightBumperButton = 6;
    public static final int backButton        = 7;
    public static final int startButton       = 8;
    public static final int leftStickButton   = 9;
    public static final int rightStickButton  = 10;
  //End of Buttons


}
