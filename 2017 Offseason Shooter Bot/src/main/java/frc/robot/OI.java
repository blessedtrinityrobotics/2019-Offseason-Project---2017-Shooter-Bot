/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.MoveToPos;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

  // Starts Xbox Controllers
  private XboxController driverController = new XboxController(RobotMap.driveControllerPort);
  private XboxController operatorController = new XboxController(RobotMap.operatorControllerPort);

  // Starts Driver Buttons
  Button xButtonDriver             = new JoystickButton(driverController, RobotMap.xButton);
  Button aButtonDriver             = new JoystickButton(driverController, RobotMap.aButton);
  Button bButtonDriver             = new JoystickButton(driverController, RobotMap.bButton);
  Button yButtonDriver             = new JoystickButton(driverController, RobotMap.yButton);
  Button backButtonDriver          = new JoystickButton(driverController, RobotMap.backButton);
  Button startButtonDriver         = new JoystickButton(driverController, RobotMap.startButton);
  Button leftBumperButtonDriver    = new JoystickButton(driverController, RobotMap.leftBumperButton);
  Button rightBumperButtonDriver   = new JoystickButton(driverController, RobotMap.rightBumperButton);
  Button leftStickButtonDriver     = new JoystickButton(driverController, RobotMap.leftStickButton);
  Button rightStickButtonDriver    = new JoystickButton(driverController, RobotMap.rightStickButton);

  // Starts Operator Buttons
  Button xButtonOperator           = new JoystickButton(operatorController, RobotMap.xButton);
  Button aButtonOperator           = new JoystickButton(operatorController, RobotMap.aButton);
  Button bButtonOperator           = new JoystickButton(operatorController, RobotMap.bButton);
  Button yButtonOperator           = new JoystickButton(operatorController, RobotMap.yButton);
  Button backButtonOperator        = new JoystickButton(operatorController, RobotMap.backButton);
  Button startButtonOperator       = new JoystickButton(operatorController, RobotMap.startButton);
  Button leftBumperButtonOperator  = new JoystickButton(operatorController, RobotMap.leftBumperButton);
  Button rightBumperButtonOperator = new JoystickButton(operatorController, RobotMap.rightBumperButton);
  Button leftStickButtonOperator   = new JoystickButton(operatorController, RobotMap.leftStickButton);
  Button rightStickButtonOperator  = new JoystickButton(operatorController, RobotMap.rightStickButton);


  public double getDriverRawAxis(int axis) {
    return driverController.getRawAxis(axis);
  }

  public double getOperatorRawAxis(int axis) {
    return operatorController.getRawAxis(axis);
  }

  public OI() {
    xButtonDriver.whenPressed(new MoveToPos(10)); // Moves 10 inches forwards
  }
  
}
