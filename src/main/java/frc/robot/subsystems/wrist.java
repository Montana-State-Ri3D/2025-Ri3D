package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class wrist extends SubsystemBase {
    private final CANSparkMax motor;
    private final SparkPIDController pidController;
    private final RelativeEncoder encoder;

    // PID coefficients
    private static final double kP = 0.1;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kIz = 0.0;
    private static final double kFF = 0.0;
    private static final double kMaxOutput = 1.0;
    private static final double kMinOutput = -1.0;

    public wrist(int motorID) {

        motor = new CANSparkMax(motorID, MotorType.kBrushless);

        pidController = motor.getPIDController();
        encoder = motor.getEncoder();

        // Set PID coefficients
        pidController.setP(kP);
        pidController.setI(kI);
        pidController.setD(kD);
        pidController.setIZone(kIz);
        pidController.setFF(kFF);
        pidController.setOutputRange(kMinOutput, kMaxOutput);
    }

    public void setPosition(double position) {
        pidController.setReference(position, CANSparkMax.ControlType.kPosition);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}