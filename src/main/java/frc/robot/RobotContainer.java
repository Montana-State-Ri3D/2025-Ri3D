// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.Arm.ArmPosition;
import frc.robot.subsystems.DriveTrain.DriveTrain;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.utilities.SubsystemFactory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveStationConstants;
import frc.robot.commands.IntakeIn;
import frc.robot.commands.IntakeOut;
import frc.robot.commands.MoveIntake;
import frc.robot.commands.MoveArm;
import frc.robot.commands.MoveElbowManual;
import frc.robot.commands.MoveElevatorManual;
import frc.robot.commands.MoveWristManual;
import frc.robot.commands.MoveArm.ArmPreset;

public class RobotContainer {
  public enum ArmPreset {
        T1(new ArmPosition(12, Units.degreesToRadians(180), Units.degreesToRadians(180))),
        T2(new ArmPosition(30, Units.degreesToRadians(210), Units.degreesToRadians(150))),
        L2_LINEUP(new ArmPosition(0, Units.degreesToRadians(227), Units.degreesToRadians(191))),
        L2_SCORE(new ArmPosition(0, Units.degreesToRadians(219),Units.degreesToRadians(208))),
        L3_LINEUP(new ArmPosition(18.5, Units.degreesToRadians(229), Units.degreesToRadians(180))),
        L3_SCORE(new ArmPosition(15.5, Units.degreesToRadians(212), Units.degreesToRadians(200))),
        L4_LINEUP(new ArmPosition(41, Units.degreesToRadians(243), Units.degreesToRadians(148))),
        L4_SCORE(new ArmPosition(38, Units.degreesToRadians(239), Units.degreesToRadians(131))),
        STORAGE(new ArmPosition(0, Units.degreesToRadians(269), Units.degreesToRadians(94))),
        CORAL_HANDOFF(new ArmPosition(30, Units.degreesToRadians(115), Units.degreesToRadians(222)));

        public final ArmPosition position;

        ArmPreset(ArmPosition position) {
            this.position = position;
        }
    }

  // Subsystems
  private DriveTrain driveTrain;
  private Intake intake;
  private Arm arm;

  // Commands
  private Command defaultDriveCommand;

  private SequentialCommandGroup T1;
  private SequentialCommandGroup T2;
  private SequentialCommandGroup L2_Lineup;
  private SequentialCommandGroup L2_SCORE;
  private SequentialCommandGroup L3_LINEUP;
  private SequentialCommandGroup L3_SCORE;
  private SequentialCommandGroup L4_LINEUP;
  private SequentialCommandGroup L4_SCORE;
  private SequentialCommandGroup storage;
  private SequentialCommandGroup coralHandoff;

  // Controllers
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

    coralHandoff = new SequentialCommandGroup();

    coralHandoff.addCommands(new InstantCommand(() -> arm.setElevatorPos(30), arm));
    coralHandoff.addCommands(new WaitCommand(5.0));
    coralHandoff.addCommands(new InstantCommand(() -> arm.setElbowPos(Units.degreesToRadians(115)), arm));
    coralHandoff.addCommands(new InstantCommand(() -> arm.setWristPos(Units.degreesToRadians(222)), arm));
  }

  /**
   * Configure the button bindings
   */
  private void configureBindings() {
    driverController.start().onTrue(new InstantCommand(() -> driveTrain.resetGyro()));
    driverController.back().onTrue(new InstantCommand(() -> driveTrain.resetPose()));
    //driverController.rightBumper().onTrue(new IntakeIn(intake, Intake.IntakePosition.ALGAE, 0.2, 0.5));
    driverController.rightBumper().onTrue(new IntakeIn(intake, Intake.IntakePosition.CORAL, 100, 0.8));
    //driverController.b().whileTrue(new IntakeOut(intake));


    //driverController.a().onTrue(new MoveIntake(intake, () -> driverController.getRightY(), () -> driverController.a().getAsBoolean()));

    driverController.x().onTrue(new InstantCommand(() -> intake.setPivotPosition(Intake.IntakePosition.CORAL)));
    driverController.y().onTrue(new InstantCommand(() -> intake.setPivotPosition(Intake.IntakePosition.HANDOFF)));
    driverController.b().onTrue(new InstantCommand(() -> intake.setPivotPosition(Intake.IntakePosition.IDLE)));

    driverController.leftBumper().onTrue(new InstantCommand(() -> arm.setElevatorLimits()));

    // driverController.x().onTrue(new MoveArm(ArmPreset.T1, () -> driverController.start().getAsBoolean(), arm));
    // driverController.y().onTrue(new MoveArm(ArmPreset.T2, () -> driverController.start().getAsBoolean(), arm));

    driverController.rightBumper().onTrue(coralHandoff);


    // uncomment to use the manual controls

    // driverController.a().onTrue(
    //   new MoveElevatorManual(
    //     arm,
    //     () -> driverController.a().getAsBoolean(),
    //     () -> driverController.getRightY()
    //   )
    // );

    driverController.a().onTrue(
      new MoveElbowManual(
        arm,
        () -> driverController.a().getAsBoolean(),
        () -> driverController.getRightY()
      )
    );

    // driverController.a().onTrue(
    //   new MoveWristManual(
    //     arm,
    //     () -> driverController.a().getAsBoolean(),
    //     () -> driverController.getRightY()
    //   )
    // );
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
