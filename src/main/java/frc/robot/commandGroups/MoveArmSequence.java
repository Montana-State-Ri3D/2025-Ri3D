// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm.Arm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveArmSequence extends SequentialCommandGroup {
  private Arm arm;

  boolean moveElevatorFirst;

  /** Creates a new MoveArmSequence. */
  public MoveArmSequence(Arm arm, boolean moveElevatorFirst) {
    this.arm = arm;
    this.moveElevatorFirst = moveElevatorFirst;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    if(moveElevatorFirst){
      addCommands(
        new InstantCommand(() -> arm.setElevatorPos(0), arm),
        new ParallelCommandGroup(
          new InstantCommand(() -> arm.setElbowPos(0.0), arm),
          new InstantCommand(() -> arm.setWristPos(0.0), arm)
        )
        );
    } else {
      addCommands(
        new ParallelCommandGroup(
          new InstantCommand(() -> arm.setElbowPos(0.0), arm),
          new InstantCommand(() -> arm.setWristPos(0.0), arm)
        ),
        new InstantCommand(() -> arm.setElevatorPos(0), arm)
      );
    }
  }
}
