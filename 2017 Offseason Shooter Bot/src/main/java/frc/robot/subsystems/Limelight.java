/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.commands.checkTarget;


import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Add your docs here.
 */
public class Limelight extends Subsystem {
  // Local Variables
  public boolean m_LimelightHasValidTarget = false;
  public double steer_cmd;
  public double drive_cmd;
  public boolean ledStatus = true;
  
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new checkTarget());
  }
  
  /**
   * This function implements a simple method of generating driving and steering commands
   * based on the tracking data from a limelight camera.
  */
  public void approachTargetWithVision() {
    final double STEER_P = 0.075;                     // how hard to turn toward the target
    final double DRIVE_P = 0.2;                     // how hard to drive fwd toward the target
    final double DESIRED_TARGET_AREA = 2.0;         // Area of the target when the robot reaches the wall
    final double MAX_DRIVE = 0.5;                   // Simple speed limit so we don't drive too fast
    final double STEER_I = 0.0;
    final double DRIVE_I = 0.0;
    final double xError;
    final double aError;
    double STEER_INTEGRAL = 0;
    double DRIVE_INTEGRAL = 0;

    double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
    xError = tx;
    aError = (DESIRED_TARGET_AREA - ta);
    SmartDashboard.putNumber("TA", ta);
    SmartDashboard.putNumber("TA Error", aError);
    SmartDashboard.putNumber("TX Error", xError);
    STEER_INTEGRAL = STEER_INTEGRAL + (xError*0.02);
    DRIVE_INTEGRAL = DRIVE_INTEGRAL + (aError * 0.02);

    if (tv < 1.0) {
      m_LimelightHasValidTarget = false;
      drive_cmd = 0.0;
      steer_cmd = 0.0;
    } else {
      m_LimelightHasValidTarget = true;
      // Start with proportional steering
      steer_cmd = (xError * STEER_P) + (STEER_INTEGRAL * STEER_I);

      // try to drive forward until the target area reaches our desired area
      drive_cmd = (aError * DRIVE_P) + (DRIVE_INTEGRAL * DRIVE_I);
      // don't let the robot drive too fast into the goal
      if (drive_cmd > MAX_DRIVE){
        drive_cmd = MAX_DRIVE;
      }
    }

    Robot.driveTrain.leftMasterMotor.set(ControlMode.PercentOutput, (drive_cmd + steer_cmd));
    Robot.driveTrain.leftSlaveMotor1.follow(Robot.driveTrain.leftMasterMotor);
    Robot.driveTrain.leftSlaveMotor2.follow(Robot.driveTrain.leftMasterMotor);
    Robot.driveTrain.rightMasterMotor.set(ControlMode.PercentOutput, (-drive_cmd - steer_cmd));
    Robot.driveTrain.rightSlaveMotor1.follow(Robot.driveTrain.rightMasterMotor);
    Robot.driveTrain.rightSlaveMotor2.follow(Robot.driveTrain.rightMasterMotor);

  }

  // Check if there is a valid vision target
  public boolean checkTarget(){
    double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    if (tv < 1.0) {
      SmartDashboard.putBoolean("Valid Vision Target", false);
      return false;
    } else {
      SmartDashboard.putBoolean("Valid Vision Target", true);
      return true;
    }
  }

  // Turn on/off Vision Tracking + LED's
  public void toggleVision(){
    if(ledStatus){ //turn off
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
      ledStatus = false;
    } else { //turn on
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
      ledStatus = true;
    }
  }
  
}
