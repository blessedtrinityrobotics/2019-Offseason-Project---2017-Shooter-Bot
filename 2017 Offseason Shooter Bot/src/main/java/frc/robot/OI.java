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

  private XboxController driverController = new XboxController(RobotMap.driveControllerPort);
  private XboxController operatorController = new XboxController(RobotMap.operatorControllerPort);
  Button xButton = new JoystickButton(driverController, RobotMap.xButton);

  public double getDriverRawAxis(int axis) {
    return driverController.getRawAxis(axis);
  }

  public double getOperatorRawAxis(int axis) {
    return operatorController.getRawAxis(axis);
  }

  public OI() {
    xButton.whenPressed(new MoveToPos(1000));
  }
  
}
