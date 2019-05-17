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
import frc.robot.commands.Shoot;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/**
 * Add your docs here.
 */
public class Shooter extends Subsystem {


  // Starts Drive Train GB Motors
  public TalonSRX shooterMasterMotor   = new TalonSRX(RobotMap.shooterMasterMotorPort);
  public VictorSPX shooterSlaveMotor  = new VictorSPX(RobotMap.shooterSlaveMotorPort);
 

  public Shooter() {

    // Configure Master Shooter Motor
    shooterMasterMotor.setInverted(false);
    shooterMasterMotor.selectProfileSlot(PIDConstants.kSlot_Shoot, PIDConstants.PID_PRIMARY);
    shooterMasterMotor.config_kP(PIDConstants.kSlot_Shoot, PIDConstants.kGains_Shoot.kP, PIDConstants.kTimeoutMs);
    shooterMasterMotor.config_kI(PIDConstants.kSlot_Shoot, PIDConstants.kGains_Shoot.kI, PIDConstants.kTimeoutMs);
    shooterMasterMotor.config_kD(PIDConstants.kSlot_Shoot, PIDConstants.kGains_Shoot.kD, PIDConstants.kTimeoutMs);
    shooterMasterMotor.config_kF(PIDConstants.kSlot_Shoot, PIDConstants.kGains_Shoot.kF, PIDConstants.kTimeoutMs);
    shooterMasterMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, PIDConstants.PID_PRIMARY, PIDConstants.kTimeoutMs);
    shooterMasterMotor.setSensorPhase(false);
    shooterMasterMotor.configOpenloopRamp(1, PIDConstants.kTimeoutMs);
    shooterMasterMotor.setNeutralMode(NeutralMode.Coast);

    // Configure Slave Shooter Motor
    shooterSlaveMotor.setInverted(false);
    shooterSlaveMotor.configOpenloopRamp(1, PIDConstants.kTimeoutMs);
    shooterSlaveMotor.setNeutralMode(NeutralMode.Coast);

  }
  
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new Shoot());
  }

  public void setShooterSpeed(double speed) {
    shooterMasterMotor.set(ControlMode.PercentOutput, speed);
    shooterSlaveMotor.follow(shooterMasterMotor);
  }
  
  
  /**
   * 
   * @param RPM RPM to spin Shooter to. Velocity Control Loop
   * 
   */
  public void spinShooterToTargetRMP(double RPM){
    shooterMasterMotor.set(ControlMode.Velocity, RPM);
    shooterSlaveMotor.follow(shooterMasterMotor);
  }
  
  /**
   * 
   * @param RPM RPM to spin Shooter to. kF Control Loop
   * 
   */
  public void shooterKfControl(double RPM){
     
  }
  


}
