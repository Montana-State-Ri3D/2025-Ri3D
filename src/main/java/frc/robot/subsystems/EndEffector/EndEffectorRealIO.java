package frc.robot.subsystems.EndEffector;

import com.revrobotics.CANSparkMax;

public class EndEffectorRealIO implements EndEffectorIO {
    private CANSparkMax motor;

    public EndEffectorRealIO(int motorId) {
        motor = new CANSparkMax(motorId, CANSparkMax.MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    @Override
    public void updateInputs(EndEffectorIOInputs inputs) {
        inputs.power = motor.get();
        inputs.current = motor.getOutputCurrent();
    }

    @Override
    public void setPower(double power) {
        motor.set(power);
    }
}
