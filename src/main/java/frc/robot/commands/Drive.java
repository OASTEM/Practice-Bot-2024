// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utils.Constants;
import frc.robot.utils.DualJoystick;

public class Drive extends Command {
  DriveTrain driveTrain;
  DualJoystick joystick;

  public Drive(DriveTrain driveTrain, DualJoystick joystick) {
    addRequirements(driveTrain);
    this.driveTrain = driveTrain;
    this.joystick = joystick;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double y, x;
    switch (driveTrain.getDriveMode()) {
      case ArcadeDrive:
        y = joystick.getLeftRawAxis(Constants.DualJoystick.Y_AXIS);
        x = joystick.getRightRawAxis(Constants.DualJoystick.X_AXIS) / 2;
        break;
      case TankDrive:
        x = joystick.getLeftRawAxis(Constants.DualJoystick.Y_AXIS);
        y = joystick.getRightRawAxis(Constants.DualJoystick.Y_AXIS);
        break;
      default:
        y = joystick.getLeftRawAxis(Constants.DualJoystick.Y_AXIS);
        x = joystick.getRightRawAxis(Constants.DualJoystick.X_AXIS) / 2;
        break;
    }

    double rightDial = joystick.getSlowMode();

    driveTrain.drive(x * rightDial, y * rightDial);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
