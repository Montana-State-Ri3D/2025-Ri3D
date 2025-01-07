package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.Arm.ArmPosition;

public class MoveArm extends Command {
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
