package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.Arm.ArmPosition;

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

    private final double WRIST_TOLEERANCE = 0.1;
    private final double ELBOW_TOLEERANCE = 0.1;
    private final double ELEVATOR_TOLEERANCE = 0.1;

    private final Arm arm;
    private final ArmPreset preset;

    public MoveArm(ArmPreset preset, Arm arm) {
        this.arm = arm;
        this.preset = preset;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        this.arm.setArmPosition(preset.position);
    }

    @Override
    public boolean isFinished() {
        ArmPosition current = arm.getPosition();
        
        return Math.abs(current.elevatorPosition - preset.position.elevatorPosition) < this.ELEVATOR_TOLEERANCE
            && Math.abs(current.elbowPosition - preset.position.elbowPosition) < this.ELBOW_TOLEERANCE
            && Math.abs(current.wristPosition - preset.position.wristPosition) < this.WRIST_TOLEERANCE;
    }
}
