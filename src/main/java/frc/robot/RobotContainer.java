// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.DriveTrain.DriveTrain;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.utilities.SubsystemFactory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveStationConstants;
import frc.robot.commands.IntakeIn;

public class RobotContainer {

  // Subsystems
  private DriveTrain driveTrain;
  private Intake intake;

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
    this.intake = factory.createIntake();

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
    driverController.rightBumper().onTrue(new IntakeIn(intake, Intake.IntakePosition.ALGAE, 0.2, 0.5));
    driverController.leftBumper().onTrue(new IntakeIn(intake, Intake.IntakePosition.CORAL, 0.2, 0.5));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
