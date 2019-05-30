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
import edu.wpi.first.networktables.NetworkTableInstance;



/**
 * Add your docs here.
 */
public class DriveTrain_Subsystem extends Subsystem {


  // Starts Drive Train GB Motors
  public TalonSRX leftMasterMotor   = new TalonSRX(RobotMap.leftMasterMotorPort);
  public VictorSPX leftSlaveMotor1  = new VictorSPX(RobotMap.leftSlaveMotor1Port);
  public VictorSPX leftSlaveMotor2  = new VictorSPX(RobotMap.leftSlaveMotor2Port);
  public TalonSRX rightMasterMotor  = new TalonSRX(RobotMap.rightMasterMotorPort);
  public VictorSPX rightSlaveMotor1 = new VictorSPX(RobotMap.rightSlaveMotor1Port);
  public VictorSPX rightSlaveMotor2 = new VictorSPX(RobotMap.rightSlaveMotor2Port);
  public ADXRS450_Gyro onboardGyro;
  public boolean m_LimelightHasValidTarget = false;
  public double m_LimelightDriveCommand = 0.0;
  public double m_LimelightSteerCommand = 0.0;
  public boolean ledStatus = true;

  public DriveTrain_Subsystem() {

    // Configure Left GB Motors
    leftMasterMotor.selectProfileSlot(PIDConstants.kSlot_Drive, PIDConstants.PID_PRIMARY);
    leftMasterMotor.config_kP(PIDConstants.kSlot_Drive, PIDConstants.kGains_Drive.kP, PIDConstants.kTimeoutMs);
    leftMasterMotor.config_kI(PIDConstants.kSlot_Drive, PIDConstants.kGains_Drive.kI, PIDConstants.kTimeoutMs);
    leftMasterMotor.config_kD(PIDConstants.kSlot_Drive, PIDConstants.kGains_Drive.kD, PIDConstants.kTimeoutMs);
    leftMasterMotor.config_kF(PIDConstants.kSlot_Drive, PIDConstants.kGains_Drive.kF, PIDConstants.kTimeoutMs);
    leftMasterMotor.configMotionAcceleration(PIDConstants.kDriveTrainAccel, PIDConstants.kTimeoutMs);
    leftMasterMotor.configMotionCruiseVelocity(PIDConstants.kDriveTrainVelocity, PIDConstants.kTimeoutMs);
    leftMasterMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, PIDConstants.PID_PRIMARY, PIDConstants.kTimeoutMs);
    leftMasterMotor.setSensorPhase(true);
    leftMasterMotor.configOpenloopRamp(1, PIDConstants.kTimeoutMs);
    leftMasterMotor.setNeutralMode(NeutralMode.Coast);
    leftSlaveMotor1.setNeutralMode(NeutralMode.Coast);
    leftSlaveMotor2.setNeutralMode(NeutralMode.Coast);

    // Configure Right GB Motors
    rightMasterMotor.selectProfileSlot(PIDConstants.kSlot_Drive, PIDConstants.PID_PRIMARY);
    rightMasterMotor.config_kP(PIDConstants.kSlot_Drive, PIDConstants.kGains_Drive.kP, PIDConstants.kTimeoutMs);
    rightMasterMotor.config_kI(PIDConstants.kSlot_Drive, PIDConstants.kGains_Drive.kI, PIDConstants.kTimeoutMs);
    rightMasterMotor.config_kD(PIDConstants.kSlot_Drive, PIDConstants.kGains_Drive.kD, PIDConstants.kTimeoutMs);
    rightMasterMotor.config_kF(PIDConstants.kSlot_Drive, PIDConstants.kGains_Drive.kF, PIDConstants.kTimeoutMs);
    rightMasterMotor.configMotionAcceleration(PIDConstants.kDriveTrainAccel, PIDConstants.kTimeoutMs);
    rightMasterMotor.configMotionCruiseVelocity(PIDConstants.kDriveTrainVelocity, PIDConstants.kTimeoutMs);
    rightMasterMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, PIDConstants.PID_PRIMARY, PIDConstants.kTimeoutMs);
    rightMasterMotor.setSensorPhase(false);
    rightMasterMotor.configOpenloopRamp(1, PIDConstants.kTimeoutMs);
    rightMasterMotor.setNeutralMode(NeutralMode.Coast);
    rightSlaveMotor1.setNeutralMode(NeutralMode.Coast);
    rightSlaveMotor2.setNeutralMode(NeutralMode.Coast);

    onboardGyro = new  ADXRS450_Gyro();
    onboardGyro.calibrate(); // Calibrates the gyro
    onboardGyro.reset();     // Sets gyro to 0 degrees

  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new Drive());
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
   * @param angle Angle to Drive Straight to
   * @param direction Direction to Drive Straight. 1.0 is Forward, -1.0 is backwards
   *  
   */
  public void driveStraight(int angle, double direction){
    double currentAngle = onboardGyro.getAngle();
    double targetAngle = angle;
    double maxSpeed = direction * 0.75;
    double kP = 0.0;
    double kI = 0.0;
    double kD = 0.0;
    double integral = 0;
    double error = targetAngle - currentAngle;
    integral = integral + (error*0.02);
    double turnCommand = (error * kP) + (integral * kI);
    leftMasterMotor.set(ControlMode.PercentOutput, (maxSpeed - turnCommand));
    leftSlaveMotor1.follow(leftMasterMotor);
    leftSlaveMotor2.follow(leftMasterMotor);
    rightMasterMotor.set(ControlMode.PercentOutput, -(maxSpeed + turnCommand));
    rightSlaveMotor1.follow(rightMasterMotor);
    rightSlaveMotor2.follow(rightMasterMotor);
  }
   
  /**
   * This function implements a simple method of generating driving and steering commands
   * based on the tracking data from a limelight camera.
   */
  public void Update_Limelight_Tracking() {
    final double STEER_P = 0.0;                     // how hard to turn toward the target
    final double DRIVE_P = 0.0;                     // how hard to drive fwd toward the target
    final double DESIRED_TARGET_AREA = 0.0;         // Area of the target when the robot reaches the wall
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
    aError = DESIRED_TARGET_AREA - ta;
    STEER_INTEGRAL = STEER_INTEGRAL + (xError*0.02);
    DRIVE_INTEGRAL = DRIVE_INTEGRAL + (aError * 0.02);

    if (tv < 1.0)
    {
      m_LimelightHasValidTarget = false;
      m_LimelightDriveCommand = 0.0;
      m_LimelightSteerCommand = 0.0;
      return;
    }

    m_LimelightHasValidTarget = true;

    // Start with proportional steering
    double steer_cmd = (tx * STEER_P) + (STEER_INTEGRAL * STEER_I);
    m_LimelightSteerCommand = steer_cmd;

    // try to drive forward until the target area reaches our desired area
    double drive_cmd = (aError * DRIVE_P) + (DRIVE_INTEGRAL * DRIVE_I);

    // don't let the robot drive too fast into the goal
    if (drive_cmd > MAX_DRIVE)
    {
      drive_cmd = MAX_DRIVE;
    }
    m_LimelightDriveCommand = drive_cmd;
  }

  public void toggleGreenLEDS(){
    // turn on/off Vision Tracking
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
