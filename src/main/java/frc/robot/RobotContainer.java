// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Drive;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Jevois;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Vision;
import frc.robot.utils.DualJoystick;
import frc.robot.utils.Constants.DriveTrain.DriveMode;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
  private DriveTrain driveTrain = new DriveTrain();
  // private Jevois jevois = new Jevois();
  private Vision averi = new Vision();

  private final DualJoystick joysticks = new DualJoystick(0, 1);
  private final JoystickButton button8 = new JoystickButton(joysticks.getRighJoystick(), 2);
  private final JoystickButton button1 = new JoystickButton(joysticks.getRighJoystick(), 1);

  // private final Drive drive = new Drive(driveTrain, joysticks);
  private final LED led = new LED();
  public RobotContainer() {
    // driveTrain.setDriveMode(DriveMode.ArcadeDrive);
    // driveTrain.setDefaultCommand(new InstantCommand());

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    button1.onTrue(new InstantCommand(driveTrain::setVelocity));
    button8.onTrue(new InstantCommand(driveTrain::stopMotor));

  }

  // public void changeDriveMode() {
  //   switch (driveTrain.getDriveMode()) {
  //     case ArcadeDrive:
  //       driveTrain.setDriveMode(DriveMode.TankDrive);

  //       break;
  //     case TankDrive:
  //       driveTrain.setDriveMode(DriveMode.ArcadeDrive);

  //       break;
  //     default:
  //       driveTrain.setDriveMode(DriveMode.ArcadeDrive);

  //       break;
  //   }

  // }

  public Command getAutonomousCommand() {
    return null;
  }
}
