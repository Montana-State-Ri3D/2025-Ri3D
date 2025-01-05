package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm.Arm;

public class MoveArm extends Command {
    public enum ArmPreset {
        L1(new ArmPosition(0, 0, 0)),
        L2(new ArmPosition(0, 0, 0)),
        L3(new ArmPosition(0, 0, 0)),
        L4(new ArmPosition(0, 0, 0));

        public final ArmPosition position;

        ArmPreset(ArmPosition position) {
            this.position = position;
        }
    }

    Arm arm;


    public MoveArm(ArmPreset preset, Arm arms) {
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        // set the target position for the arm
    }

    @Override
    public boolean isFinished() {
        // return true if the arm is at the target position
        return true;
    }
}
