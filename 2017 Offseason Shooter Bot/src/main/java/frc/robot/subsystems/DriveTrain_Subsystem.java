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
import frc.robot.commands.TankDrive;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/**
 * Add your docs here.
 */
public class DriveTrain_Subsystem extends Subsystem {


  // Starts Drive Train GB Motors
  public TalonSRX leftMasterMotor = new TalonSRX(RobotMap.leftMasterMotorPort);
  public VictorSPX leftSlaveMotor1 = new VictorSPX(RobotMap.leftSlaveMotor1Port);
  public VictorSPX leftSlaveMotor2 = new VictorSPX(RobotMap.leftSlaveMotor2Port);
  public TalonSRX rightMasterMotor = new TalonSRX(RobotMap.rightMasterMotorPort);
  public VictorSPX rightSlaveMotor1 = new VictorSPX(RobotMap.rightSlaveMotor1Port);
  public VictorSPX rightSlaveMotor2 = new VictorSPX(RobotMap.rightSlaveMotor2Port);

  public DriveTrain_Subsystem() {
    leftMasterMotor.selectProfileSlot(PIDConstants.kSlot_Drive, PIDConstants.PID_PRIMARY);
    leftMasterMotor.config_kP(PIDConstants.kSlot_Drive, PIDConstants.kGains_Drive.kP, PIDConstants.kTimeoutMs);
    leftMasterMotor.config_kI(PIDConstants.kSlot_Drive, PIDConstants.kGains_Drive.kI, PIDConstants.kTimeoutMs);
    leftMasterMotor.config_kD(PIDConstants.kSlot_Drive, PIDConstants.kGains_Drive.kD, PIDConstants.kTimeoutMs);
    leftMasterMotor.config_kF(PIDConstants.kSlot_Drive, PIDConstants.kGains_Drive.kF, PIDConstants.kTimeoutMs);
    leftMasterMotor.configMotionAcceleration(PIDConstants.kDriveTrainAccel, PIDConstants.kTimeoutMs);
    leftMasterMotor.configMotionCruiseVelocity(PIDConstants.kDriveTrainVelocity, PIDConstants.kTimeoutMs);

    rightMasterMotor.selectProfileSlot(PIDConstants.kSlot_Drive, PIDConstants.PID_PRIMARY);
    rightMasterMotor.config_kP(PIDConstants.kSlot_Drive, PIDConstants.kGains_Drive.kP, PIDConstants.kTimeoutMs);
    rightMasterMotor.config_kI(PIDConstants.kSlot_Drive, PIDConstants.kGains_Drive.kI, PIDConstants.kTimeoutMs);
    rightMasterMotor.config_kD(PIDConstants.kSlot_Drive, PIDConstants.kGains_Drive.kD, PIDConstants.kTimeoutMs);
    rightMasterMotor.config_kF(PIDConstants.kSlot_Drive, PIDConstants.kGains_Drive.kF, PIDConstants.kTimeoutMs);
    rightMasterMotor.configMotionAcceleration(PIDConstants.kDriveTrainAccel, PIDConstants.kTimeoutMs);
    rightMasterMotor.configMotionCruiseVelocity(PIDConstants.kDriveTrainVelocity, PIDConstants.kTimeoutMs);
  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new TankDrive());
  }

  public void setLeftMotors(double speed) {
    leftMasterMotor.set(ControlMode.PercentOutput, -speed);
    leftSlaveMotor1.follow(leftMasterMotor);
    leftSlaveMotor2.follow(leftMasterMotor);
  }

  public void setRightMotors(double speed) {
    rightMasterMotor.set(ControlMode.PercentOutput, speed);
    rightSlaveMotor1.follow(rightMasterMotor);
    rightSlaveMotor2.follow(rightMasterMotor);
  }

  public void moveToPos(double pos){
    leftMasterMotor.set(ControlMode.MotionMagic, -pos);
    leftSlaveMotor1.follow(leftMasterMotor);
    leftSlaveMotor2.follow(leftMasterMotor);
    rightMasterMotor.set(ControlMode.MotionMagic, pos);
    rightSlaveMotor1.follow(rightMasterMotor);
    rightSlaveMotor2.follow(rightMasterMotor);
  }

  public double getAvgCurrentPosition(){
    return ((leftMasterMotor.getSelectedSensorPosition() + rightMasterMotor.getSelectedSensorPosition())/2);
  }
   

}
