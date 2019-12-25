/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
//import frc.robot.commands.IntakeBalls;


/**
 * Add your docs here.
 */
public class Intake extends Subsystem {
   
  // Starts Intake Motors
  public VictorSP leftIntakeMotor = new VictorSP(RobotMap.leftIntakeMotorPort);
  public VictorSP rightIntakeMotor = new VictorSP(RobotMap.rightIntakeMotorPort);
  
  public Intake() {
    // Configure Intake Motors
    leftIntakeMotor.setInverted(true); // Reverse direction
    rightIntakeMotor.setInverted(false); // !Reverse direction
  } 
  
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    //setDefaultCommand(new IntakeBalls());
  }
  
  // Set Intake Motors Speed
  public void setIntakeSpeed(double speed) {
    leftIntakeMotor.set(speed);
    rightIntakeMotor.set(speed);
  }
  
}
