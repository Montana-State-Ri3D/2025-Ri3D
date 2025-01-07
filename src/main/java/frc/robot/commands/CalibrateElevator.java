package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm.Arm;

public class CalibrateElevator extends Command {

    private Arm arm;

    public CalibrateElevator(Arm arm) {
        this.arm = arm;
        addRequirements(arm);
    }

    // TODO: ensure correct direction
    @Override
    public void initialize() {
        this.arm.setElevatorPower(-0.1);
    }

    @Override
    public boolean isFinished() {
        return this.arm.limitSwitchHit();
    }

    @Override
    public void end(boolean interrupted) {
        this.arm.setElevatorPower(0);
        this.arm.setElevatorLimits();
    }
}
