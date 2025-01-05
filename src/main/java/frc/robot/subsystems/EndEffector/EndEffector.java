package frc.robot.subsystems.EndEffector;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class EndEffector extends SubsystemBase {
    private final int CURRENT_SPIKE_THRESHOLD = 1;

    private final EndEffectorIO io;
    private final EndEffectorIOInputsAutoLogged inputs = new EndEffectorIOInputsAutoLogged();

    public EndEffector(EndEffectorIO io) {
        this.io = io;
        io.updateInputs(inputs);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        Logger.processInputs("EndEffector", inputs);
        Logger.recordOutput("EndEffector/CurrentCommand",
                this.getCurrentCommand() != null ? this.getCurrentCommand().getName() : "none");
    }

    public void setPower(double power) {
        io.setPower(power);
    }

    public boolean currentSpike() {
        return inputs.current > CURRENT_SPIKE_THRESHOLD;
    }
}
