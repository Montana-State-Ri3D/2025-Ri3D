package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm.Arm;

public class MoveElbowManual extends Command {
    private final Arm arm;
    private final BooleanSupplier held;
    private final DoubleSupplier input;

    public MoveElbowManual(Arm arm, BooleanSupplier held, DoubleSupplier input) {
        this.arm = arm;
        this.input = input;
        this.held = held;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        double power = - 0.2 * input.getAsDouble();
        arm.setElbowPower(power);
    }

    @Override
    public boolean isFinished() {
        return !held.getAsBoolean();
    }

    @Override
    public void end(boolean interrupted) {
        arm.stop();
    }
}