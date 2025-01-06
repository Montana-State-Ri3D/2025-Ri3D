// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.utilities.Joystick;

public class MoveIntake extends Command {
  /** Creates a new MoveIntake. */

  Intake intake;
  BooleanSupplier button;
  DoubleSupplier joystick;

  public MoveIntake(Intake intake, DoubleSupplier joystick, BooleanSupplier button) {
    this.intake = intake;
    this.button = button;
    this.joystick = joystick;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setPivotPower(-Joystick.JoystickInput(joystick.getAsDouble(), 1, 0.02, 0.0));;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setPivotPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !button.getAsBoolean();
  }
}
