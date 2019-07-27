/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class VisionApproach extends Command {
  double waitTime;
  public VisionApproach(/*double time*/) {
    //waitTime = time;
    requires(Robot.limelight);
    requires(Robot.driveTrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (Robot.limelight.ledStatus == false){
      Robot.limelight.toggleVision();
    }
    Robot.limelight.approachTargetWithVision();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    /*if((Robot.driveTrain.getAvgSpeed() == 0) && ( (Robot.driveTrain.getAvgGyroAngle(waitTime) >= (Robot.driveTrain.initGyroAngle - 0.5)) && (Robot.driveTrain.getAvgGyroAngle(waitTime) <= (Robot.driveTrain.finalGyroAngle + 0.5)) ) ){
      return true;
    } else {
      return false;
    }
    */
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {

  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
