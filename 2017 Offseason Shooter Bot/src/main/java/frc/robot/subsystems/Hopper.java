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
import frc.robot.commands.FeedBalls;


/**
 * Add your docs here.
 */
public class Hopper extends Subsystem {
  
  // Starts Hopper Motors
  public VictorSP frontHopperMotor = new VictorSP(RobotMap.frontHopperMotorPort);
  public VictorSP backHopperMotor = new VictorSP(RobotMap.backHopperMotorPort);
  
  public Hopper() {
    // Configure Hopper Motors
    frontHopperMotor.setInverted(true); // Reverse direction
    backHopperMotor.setInverted(false); // !Reverse direction 
  }
  
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    //setDefaultCommand(new FeedBalls());
  }
  
  // Set Hopper Motors Speed
  public void setHopperSpeed(double speed) {
    frontHopperMotor.set(speed);
    backHopperMotor.set(speed);
  }

  // Set Bottom Rollers Speed
  public void setBottomHopperSpeed(double speed) {
    frontHopperMotor.set(speed);
  }
  
}
