package frc.robot.subsystems.EndEffector;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndEffector extends SubsystemBase {
    private final EndEffectorIO io;
    
    private final EndEffectorIOInputsAutoLogged inputs = new EndEffectorIOInputsAutoLogged();

    public EndEffector(EndEffectorIO io) {
        this.io = io;
        io.updateInputs(inputs);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public void setPower(double power) {
        inputs.power = power;
    }
}
