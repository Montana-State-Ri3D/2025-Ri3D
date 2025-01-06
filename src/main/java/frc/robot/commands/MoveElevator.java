package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Arm.Arm;

public class MoveElevator extends Command {
    private final Arm arm;
    private final CommandXboxController controller;
    private final BooleanSupplier cancel;

    public MoveElevator(Arm arm, BooleanSupplier cancel, CommandXboxController controller) {
        this.arm = arm;
        this.controller = controller;
        this.cancel = cancel;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        double input = controller.getRightY();
        input = Math.max(-0.2, Math.min(0.2, input));
        arm.setElevatorPower(input);
    }

    @Override
    public boolean isFinished() {
        return cancel.getAsBoolean() || !controller.a().getAsBoolean();
    }

    @Override
    public void end(boolean interrupted) {
        arm.stop();
    }
}