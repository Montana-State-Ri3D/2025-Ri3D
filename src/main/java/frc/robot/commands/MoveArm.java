package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.Arm.ArmPosition;

public class MoveArm extends Command {
    public enum ArmPreset {
        L1(new ArmPosition(0.1, 0, 0)),
        L2(new ArmPosition(0.9, 0, 0)),
        L3(new ArmPosition(0, 0, 0)),
        L4(new ArmPosition(0, 0, 0)),
        ALGAE(new ArmPosition(0, 0, 0)),
        CORAL(new ArmPosition(0, 0, 0));

        public final ArmPosition position;

        ArmPreset(ArmPosition position) {
            this.position = position;
        }
    }

    private final double WRIST_TOLEERANCE = 0.01;
    private final double ELBOW_TOLEERANCE = 0.01;
    private final double ELEVATOR_TOLEERANCE = 0.01;

    private final Arm arm;
    private final ArmPreset preset;

    private final BooleanSupplier cancel;

    public MoveArm(ArmPreset preset, BooleanSupplier cancel, Arm arm) {
        this.arm = arm;
        this.preset = preset;
        this.cancel = cancel;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        this.arm.setArmPosition(preset.position);
    }

    public void end(boolean interrupted) {
        this.arm.stop();
    }

    @Override
    public boolean isFinished() {
        ArmPosition current = arm.getPosition();
        
        return 
            cancel.getAsBoolean()
            || (Math.abs(current.elevatorPosition - preset.position.elevatorPosition) < this.ELEVATOR_TOLEERANCE
            && Math.abs(current.elbowPosition - preset.position.elbowPosition) < this.ELBOW_TOLEERANCE
            && Math.abs(current.wristPosition - preset.position.wristPosition) < this.WRIST_TOLEERANCE);
    }
}
