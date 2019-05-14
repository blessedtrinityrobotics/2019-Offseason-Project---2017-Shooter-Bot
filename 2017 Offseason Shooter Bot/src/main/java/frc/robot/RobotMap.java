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
  
  //Motor Ports
  public static final int
    placeholder = 69,
    //Left Drive Train GB Motors 
    leftMasterMotor = 2,
    leftSlaveMotor1 = 3,
    leftSlaveMotor2 = 4,
    //Right Drive Train GB Motors
    rightMasterMotor = 5,
    rightSlaveMotor1 = 6,
    rightSlaveMotor2 = 7,
    //Left Intake Motor
    leftIntakeMotor = 8,
    //Right Intake Motor
    rightIntakeMotor = 9,
    //Front Hopper Motor
    frontHopperMotor = 10,
    //Back Hopper Motor
    backHopperMotor = 11,
    //Shooter Motors
    shooterMasterMotor = 12,
    shooterSlaveMotor = 13;
}
