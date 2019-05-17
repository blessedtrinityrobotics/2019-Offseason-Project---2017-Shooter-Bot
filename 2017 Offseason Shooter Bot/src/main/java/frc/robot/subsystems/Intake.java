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
import frc.robot.commands.IntakeBalls;


import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

/**
 * Add your docs here.
 */
public class Intake extends Subsystem {
   
  public VictorSPX leftIntakeMotor = new VictorSPX(RobotMap.leftIntakeMotorPort);
  public VictorSPx rightIntakeMotor = new VictorSPX(RobotMap.rightIntakeMotorPort);
  
  public Intake() {
    // Configure Intake Motors
    leftIntakeMotor.setInverted(true); // Reverse direction
    leftIntakeMotor.setNeutralMode(NeutralMode.Brake);
    rightIntakeMotor.setInverted(false);
    rightIntakeMotor.setNeutralMode(NeutralMode.Brake);
  }
  
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new IntakeBalls());
  }
  
  public void setIntakeSpeed(double speed) {
    leftIntakeMotor.set(ControlMode.PercentOutput, speed);
    rightIntakeMotor.set(ControlMode.PercentOutput, speed);
  }
  
}
