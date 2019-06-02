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
import frc.robot.commands.Drive;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;




public class DriveTrain_Subsystem extends Subsystem {

  // Starts Drive Train GB Motors
  public TalonSRX leftMasterMotor   = new TalonSRX(RobotMap.leftMasterMotorPort);
  public VictorSPX leftSlaveMotor1  = new VictorSPX(RobotMap.leftSlaveMotor1Port);
  public VictorSPX leftSlaveMotor2  = new VictorSPX(RobotMap.leftSlaveMotor2Port);
  public TalonSRX rightMasterMotor  = new TalonSRX(RobotMap.rightMasterMotorPort);
  public VictorSPX rightSlaveMotor1 = new VictorSPX(RobotMap.rightSlaveMotor1Port);
  public VictorSPX rightSlaveMotor2 = new VictorSPX(RobotMap.rightSlaveMotor2Port);
  // Starts Gyro
  public ADXRS450_Gyro onboardGyro  = new  ADXRS450_Gyro();
  

  public DriveTrain_Subsystem() {

    // Configure Left GB Motors
    leftMasterMotor.selectProfileSlot(PIDConstants.kSlot_Drive, PIDConstants.PID_PRIMARY); // Profile Slot for PID Values
    leftMasterMotor.config_kP(PIDConstants.kSlot_Drive, PIDConstants.kGains_Drive.kP, PIDConstants.kTimeoutMs); // P Value
    leftMasterMotor.config_kI(PIDConstants.kSlot_Drive, PIDConstants.kGains_Drive.kI, PIDConstants.kTimeoutMs); // I Value
    leftMasterMotor.config_kD(PIDConstants.kSlot_Drive, PIDConstants.kGains_Drive.kD, PIDConstants.kTimeoutMs); // D Value
    leftMasterMotor.config_kF(PIDConstants.kSlot_Drive, PIDConstants.kGains_Drive.kF, PIDConstants.kTimeoutMs); // F Value
    leftMasterMotor.configMotionAcceleration(PIDConstants.kDriveTrainAccel, PIDConstants.kTimeoutMs); // Motion Magic Acceleration Value
    leftMasterMotor.configMotionCruiseVelocity(PIDConstants.kDriveTrainVelocity, PIDConstants.kTimeoutMs); // Motion Magic Velocity Value
    leftMasterMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, PIDConstants.PID_PRIMARY, PIDConstants.kTimeoutMs); // Select Sensor (Encoder)
    leftMasterMotor.setSensorPhase(true); // Reverse Direction of encoder
    leftMasterMotor.configOpenloopRamp(1, PIDConstants.kTimeoutMs); // % Ramp - 1 sec to full throtle
    leftMasterMotor.setNeutralMode(NeutralMode.Coast); // Neutral Mode - Coast
    leftSlaveMotor1.setNeutralMode(NeutralMode.Coast); // Neutral Mode - Coast
    leftSlaveMotor2.setNeutralMode(NeutralMode.Coast); // Neutral Mode - Coast

    // Configure Right GB Motors
    rightMasterMotor.selectProfileSlot(PIDConstants.kSlot_Drive, PIDConstants.PID_PRIMARY); // Profile Slot for PID Values
    rightMasterMotor.config_kP(PIDConstants.kSlot_Drive, PIDConstants.kGains_Drive.kP, PIDConstants.kTimeoutMs); // P Value
    rightMasterMotor.config_kI(PIDConstants.kSlot_Drive, PIDConstants.kGains_Drive.kI, PIDConstants.kTimeoutMs); // I Value
    rightMasterMotor.config_kD(PIDConstants.kSlot_Drive, PIDConstants.kGains_Drive.kD, PIDConstants.kTimeoutMs); // D Value
    rightMasterMotor.config_kF(PIDConstants.kSlot_Drive, PIDConstants.kGains_Drive.kF, PIDConstants.kTimeoutMs); // F Value
    rightMasterMotor.configMotionAcceleration(PIDConstants.kDriveTrainAccel, PIDConstants.kTimeoutMs); // Motion Magic Acceleration Value
    rightMasterMotor.configMotionCruiseVelocity(PIDConstants.kDriveTrainVelocity, PIDConstants.kTimeoutMs); // Motion Magic Velocity Value
    rightMasterMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, PIDConstants.PID_PRIMARY, PIDConstants.kTimeoutMs);// Select Sensor (Encoder)
    rightMasterMotor.setSensorPhase(false); // !Reverse Direction of encoder
    rightMasterMotor.configOpenloopRamp(1, PIDConstants.kTimeoutMs); // % Ramp - 1 sec to full throtle
    rightMasterMotor.setNeutralMode(NeutralMode.Coast); // Neutral Mode - Coast
    rightSlaveMotor1.setNeutralMode(NeutralMode.Coast); // Neutral Mode - Coast
    rightSlaveMotor2.setNeutralMode(NeutralMode.Coast); // Neutral Mode - Coast

    // Calibrates and Resets Gyro
    onboardGyro.calibrate(); // Calibrates the gyro
    onboardGyro.reset();     // Sets gyro to 0 degrees

  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new Drive());
  }

  // Set Left Motors Speed (%)
  public void setLeftMotors(double speed) {
    leftMasterMotor.set(ControlMode.PercentOutput, -speed);
    leftSlaveMotor1.follow(leftMasterMotor);
    leftSlaveMotor2.follow(leftMasterMotor);
  }

  // Set Right Motors Speed (%)
  public void setRightMotors(double speed) {
    rightMasterMotor.set(ControlMode.PercentOutput, speed);
    rightSlaveMotor1.follow(rightMasterMotor);
    rightSlaveMotor2.follow(rightMasterMotor);
  }

  /**
   * 
   * @param distance Inches to move forward or backwards
   * 
   */
  public void moveToPos(double distance){
    double encoderTarget;
    encoderTarget = distance * RobotMap.PPI;
    leftMasterMotor.set(ControlMode.MotionMagic, -encoderTarget);
    leftSlaveMotor1.follow(leftMasterMotor);
    leftSlaveMotor2.follow(leftMasterMotor);
    rightMasterMotor.set(ControlMode.MotionMagic, encoderTarget);
    rightSlaveMotor1.follow(rightMasterMotor);
    rightSlaveMotor2.follow(rightMasterMotor);
  }

  // Get Averaged Current Position of Drive Train
  public double getAvgCurrentPosition(){
    return ((leftMasterMotor.getSelectedSensorPosition() + rightMasterMotor.getSelectedSensorPosition())/2);
  }

  // Sets gyro to 0 degrees
  public void resetGyro(){
    onboardGyro.reset();     
  }

  /**
   * 
   * @param angle Angle to drive straight to
   * @param direction Direction to drive straight; 1.0 is Forward, -1.0 is backwards
   *  
   */
  public void driveStraight(double angle, double direction){
    double currentAngle = onboardGyro.getAngle();
    double targetAngle = angle;
    double maxSpeed = direction * 0.75;
    double kP = 0.0;
    double kI = 0.0;
    double kD = 0.0;
    double integral = 0;
    double derivative = 0;
    double previousError = 0;
    double error = targetAngle - currentAngle;
    integral = integral + (error*0.02);
    derivative = ((error - previousError)/0.02);
    double turnCommand = (error * kP) + (integral * kI) + (derivative * kD);
    previousError = error;
    leftMasterMotor.set(ControlMode.PercentOutput, (maxSpeed - turnCommand));
    leftSlaveMotor1.follow(leftMasterMotor);
    leftSlaveMotor2.follow(leftMasterMotor);
    rightMasterMotor.set(ControlMode.PercentOutput, -(maxSpeed + turnCommand));
    rightSlaveMotor1.follow(rightMasterMotor);
    rightSlaveMotor2.follow(rightMasterMotor);
  }
   
  

}
