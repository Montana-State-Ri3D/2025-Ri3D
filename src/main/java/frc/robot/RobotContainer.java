// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.DriveTrain.DriveTrain;
import frc.robot.utilities.SubsystemFactory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveStationConstants;
import frc.robot.commands.MoveArm;
import frc.robot.commands.MoveArm.ArmPreset;

public class RobotContainer {

  // Subsystems
  private DriveTrain driveTrain;
  private Arm arm;

  // Commands
  private Command defaultDriveCommand;

  private CommandXboxController driverController;

  public RobotContainer() {
    this.driverController = new CommandXboxController(DriveStationConstants.DRIVE_CONTROLLER_PORT_ID);

    createSubsystems();
    createCommands();
    configureBindings();
  }

  /**
   * Create all subsystems using the SubsystemFactory
   */
  private void createSubsystems() {

    SubsystemFactory factory = new SubsystemFactory();
    this.driveTrain = factory.createDriveTrain();
    this.arm = factory.createArm();
  }

  /**
   * Create all commands
   */
  private void createCommands() {
    defaultDriveCommand = driveTrain.arcadeDriveCommand(
        () -> driverController.getRightTriggerAxis() - driverController.getLeftTriggerAxis(),
        () -> driverController.getLeftX());

    driveTrain.setDefaultCommand(defaultDriveCommand);
  }

  /**
   * Configure the button bindings
   */
  private void configureBindings() {
    driverController.start().onTrue(new InstantCommand(() -> driveTrain.resetGyro()));
    driverController.back().onTrue(new InstantCommand(() -> driveTrain.resetPose()));

    driverController.a().onTrue(new MoveArm(ArmPreset.L1, () -> driverController.start().getAsBoolean(), arm));
    driverController.b().onTrue(new MoveArm(ArmPreset.L2, () -> driverController.start().getAsBoolean(), arm));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
