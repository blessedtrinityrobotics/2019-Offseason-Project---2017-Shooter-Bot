/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.RobotMap;

public class ShootingProcedure extends CommandGroup {
  
  public ShootingProcedure() {
    //Sequential Commands
    addSequential(new VisionApproach(RobotMap.waitTime)); // Approaches Target if it has not done so already
    // Parallel Commands
    addParallel(new StayOnTarget());      // Stays locked onto the target  
    addParallel(new IntakeBalls());       // Intake Runs to feed balls into hopper
    addParallel(new FeedBalls());         // Hopper Feeds Balls into shooter
    addParallel(new Shoot(RobotMap.RPM)); // Shooter Activates 
  }
}
