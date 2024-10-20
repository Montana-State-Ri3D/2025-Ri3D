// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.DriveTrain.DriveTrain;
import frc.robot.utilities.SubsystemFactory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveStationConstants;

public class RobotContainer {

  // Subsystems
  private DriveTrain driveTrain;

  // Commands
  private Command defaultDriveCommand;

  private CommandXboxController driverController;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    this.driverController = new CommandXboxController(DriveStationConstants.DRIVE_CONTROLLER_PORT_ID);

    createSubsystems();
    createCommands();
    configureBindings();
  }

  private void createSubsystems() {

    SubsystemFactory factory = new SubsystemFactory();
    this.driveTrain = factory.createDriveTrain();

  }

  private void createCommands() {
    defaultDriveCommand = driveTrain.arcadeDriveCommand(
        () -> driverController.getRightTriggerAxis() - driverController.getLeftTriggerAxis(),
        () -> driverController.getLeftX());

    driveTrain.setDefaultCommand(defaultDriveCommand);
  }

  private void configureBindings() {
    driverController.start().onTrue(new InstantCommand(() -> driveTrain.resetGyro()));
    driverController.back().onTrue(new InstantCommand(() -> driveTrain.resetPose()));
  }

  public Command getAutonomousCommand() {

    return null;
  }
}
