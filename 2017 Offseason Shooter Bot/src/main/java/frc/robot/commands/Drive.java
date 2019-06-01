/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class Drive extends Command {
  public Drive() {
    requires(Robot.driveTrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    //Get All Axis Values
    //double leftStickY = Robot.m_oi.getDriverRawAxis(RobotMap.leftStickY);
    double rightStickY = Robot.m_oi.getDriverRawAxis(RobotMap.rightStickY);
    double leftStickX = Robot.m_oi.getDriverRawAxis(RobotMap.leftStickX);
    //double rightStickX = Robot.m_oi.getDriverRawAxis(RobotMap.rightStickX);
    //double leftTrigger = Robot.m_oi.getDriverRawAxis(RobotMap.leftTriggerAxis);
    //double rightTrigger = Robot.m_oi.getDriverRawAxis(RobotMap.rightTriggerAxis);
    
  /*
    // Tank Drive Options
      // Regular Tank Drive
      Robot.driveTrain.setLeftMotors(leftStickY);
      Robot.driveTrain.setRightMotors(rightStickY);
      // Exponential Tank Drive
      Robot.driveTrain.setLeftMotors(leftStickY * Math.abs(leftStickY));
      Robot.driveTrain.setRightMotors(rightStickY * Math.abs(rightStickY));
  */
    // Split Arcade Drive Options  
      // Regular Split Arcade
      Robot.driveTrain.setLeftMotors(rightStickY + leftStickX * RobotMap.turningPower);
      Robot.driveTrain.setRightMotors(rightStickY - leftStickX * RobotMap.turningPower);
    /*
      // Exponential Drive Split Arcade
      Robot.driveTrain.setLeftMotors((rightStickY * Math.abs(rightStickY)) + (leftStickX * RobotMap.turningPower));
      Robot.driveTrain.setRightMotors((rightStickY * Math.abs(rightStickY)) - (leftStickX * RobotMap.turningPower));
      // Exponential Drive and Turn Split Arcade 
      Robot.driveTrain.setLeftMotors((rightStickY * Math.abs(rightStickY)) + ((leftStickX * Math.abs(leftStickX)) * RobotMap.turningPower));
      Robot.driveTrain.setRightMotors((rightStickY * Math.abs(rightStickY)) - ((leftStickX * Math.abs(leftStickX)) * RobotMap.turningPower));
    */
  /*
    // GTA Drive Options
      // Regular GTA Drive
      double triggerValue1 = rightTrigger - leftTrigger;
      double turnValue1 = leftStickX * RobotMap.turningPower;
      Robot.driveTrain.setLeftMotors(triggerValue1 + turnValue1);
      Robot.driveTrain.setRightMotors(triggerValue1 - turnValue1);
      // Exponential Drive GTA Drive
      double triggerValue2 = (rightTrigger - leftTrigger) * Math.abs((rightTrigger - leftTrigger));
      double turnValue2 = leftStickX * RobotMap.turningPower;
      Robot.driveTrain.setLeftMotors(triggerValue2 + turnValue2);
      Robot.driveTrain.setRightMotors(triggerValue2 - turnValue2);
      // Exponential Drive and Turn GTA Drive
      double triggerValue3 = (rightTrigger - leftTrigger) * Math.abs((rightTrigger - leftTrigger));
      double turnValue3 = (leftStickX * Math.abs(leftStickX)) * RobotMap.turningPower;
      Robot.driveTrain.setLeftMotors(triggerValue3 + turnValue3);
      Robot.driveTrain.setRightMotors(triggerValue3 - turnValue3);
  */
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.driveTrain.setLeftMotors(0);
    Robot.driveTrain.setRightMotors(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    this.end();
  }
}
