/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.PIDConstants;
import frc.robot.RobotMap;
import frc.robot.commands.FeedBalls;


import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

/**
 * Add your docs here.
 */
public class Hopper extends Subsystem {
   
  public VictorSPX frontHopperMotor = new VictorSPX(RobotMap.frontHopperMotorPort);
  public VictorSPx backHopperMotor = new VictorSPX(RobotMap.backHopperMotorPort);
  
  public Hopper() {
    // Configure Intake Motors
    frontHopperMotor.setInverted(true); // Reverse direction
    frontHopperMotor.setNeutralMode(NeutralMode.Brake);
    backHopperMotor.setInverted(false);
    backHopperMotor.setNeutralMode(NeutralMode.Brake);
  }
  
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new FeedBalls());
  }
  
  public void setHopperSpeed(double speed) {
    frontHopperMotor.set(ControlMode.PercentOutput, speed);
    backHopperMotor.set(ControlMode.PercentOutput, speed);
  }
  
}
