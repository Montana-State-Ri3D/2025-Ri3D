// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector.EndEffector;

public class GrabCoral extends Command {
    EndEffector endEffector;

    private long initTime;
    private long duration;

  /** Creates a new GrabCoral. */
  public GrabCoral(EndEffector endEffector) {
    this.endEffector = endEffector;

    initTime = -1;
    duration = 500;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(endEffector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initTime = System.currentTimeMillis();
    endEffector.setPower(1.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    endEffector.setPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    long currentTime = System.currentTimeMillis();
    // System.out.println("Time threshold true: " + (initTime > currentTime + duration));
    // System.out.println("Current threshold true: " + endEffector.currentSpike());

    return (currentTime > initTime + duration) && endEffector.currentSpike();
    
  }
}
