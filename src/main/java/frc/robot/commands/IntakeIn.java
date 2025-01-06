// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.Intake.IntakePosition;

public class IntakeIn extends Command {
  private final Intake intake;

  private IntakePosition target;
  private double speed;
  private double amps;

  private long initTime = -1;
  private long duration = 500;

  /** Creates a new IntakeIn. */
  public IntakeIn(Intake intake, IntakePosition target, double amps, double speed) {
    this.intake = intake;

    this.target = target;
    this.amps = amps;
    this.speed = speed;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished();
    initTime = System.currentTimeMillis();
    intake.setPivotPosition(target);
    intake.setPower(speed, speed - 0.2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(target == IntakePosition.ALGAE) {
      intake.pickupAlgae();
    }
    intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    long currentTime = System.currentTimeMillis();
    return (intake.hasObject() || 
            ((currentTime >= initTime + duration) && (intake.getLeftCurrent() > amps) || (intake.getRightCurrent() > amps)));
  }
}
