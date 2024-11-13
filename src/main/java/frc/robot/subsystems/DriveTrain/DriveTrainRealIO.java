package frc.robot.subsystems.DriveTrain;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import com.ctre.phoenix6.hardware.Pigeon2;
import frc.robot.Constants.DriveTrainConstants;

public class DriveTrainRealIO implements DriveTrainIO {

    private CANSparkMax leftMotorBack;
    private CANSparkMax leftMotorFront;
    private CANSparkMax rightMotorBack;
    private CANSparkMax rightMotorFront;

    private RelativeEncoder leftEncoderBack;
    private RelativeEncoder leftEncoderFront;
    private RelativeEncoder rightEncoderBack;
    private RelativeEncoder rightEncoderFront;
    
    private CANSparkMax[] Motors;
    private RelativeEncoder[] Encoders;
    private Pigeon2 pidgey;


    public DriveTrainRealIO(int leftMotorBack, int leftMotorFront, int rightMotorBack, int rightMotorFront, int pigeonID) {

        this.leftMotorBack = new CANSparkMax(leftMotorBack, CANSparkMax.MotorType.kBrushless);
        this.leftMotorFront = new CANSparkMax(leftMotorFront, CANSparkMax.MotorType.kBrushless);
        this.rightMotorBack = new CANSparkMax(rightMotorBack, CANSparkMax.MotorType.kBrushless);
        this.rightMotorFront = new CANSparkMax(rightMotorFront, CANSparkMax.MotorType.kBrushless);

        this.Motors = new CANSparkMax[4];

        this.Motors[0] = this.leftMotorBack;
        this.Motors[1] = this.leftMotorFront;
        this.Motors[2] = this.rightMotorBack;
        this.Motors[3] = this.rightMotorFront;

        this.pidgey = new Pigeon2(pigeonID);

        for (CANSparkMax motor : this.Motors) {
            motor.restoreFactoryDefaults();
            motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
            motor.setSmartCurrentLimit(40);
        }

        this.leftMotorFront.setInverted(true);
        this.rightMotorFront.setInverted(false);

        this.leftMotorBack.follow(this.leftMotorFront, false);
        this.rightMotorBack.follow(this.rightMotorFront, false);

        this.leftEncoderBack = this.leftMotorBack.getEncoder();
        this.leftEncoderFront = this.leftMotorFront.getEncoder();
        this.rightEncoderBack = this.rightMotorBack.getEncoder();
        this.rightEncoderFront = this.rightMotorFront.getEncoder();

        this.Encoders = new RelativeEncoder[4];
        this.Encoders[0] = this.leftEncoderBack;
        this.Encoders[1] = this.leftEncoderFront;
        this.Encoders[2] = this.rightEncoderBack;
        this.Encoders[3] = this.rightEncoderFront;


        for (RelativeEncoder encoder : this.Encoders) {
            encoder.setPositionConversionFactor(
                    DriveTrainConstants.DRIVE_WHEEL_RADIUS * 2.0 * Math.PI * DriveTrainConstants.DRIVE_RATIO);
            encoder.setVelocityConversionFactor(
                    DriveTrainConstants.DRIVE_WHEEL_RADIUS * 2.0 * Math.PI * DriveTrainConstants.DRIVE_RATIO / 60.0);
        }

    }

    public void drive(double left, double right) {
        this.leftMotorFront.set(left);
        this.rightMotorFront.set(right);
    }

    public void toggleMode() {
        if (this.leftMotorFront.getIdleMode() == CANSparkMax.IdleMode.kBrake) {
            for (CANSparkMax motor : this.Motors) {
                motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
            }
        } else {
            for (CANSparkMax motor : this.Motors) {
                motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
            }
        }
    }

    public void updateInputs(DriveTrainIOInputs inputs) {
        inputs.leftPower = this.leftMotorFront.get();
        inputs.rightPower = this.rightMotorFront.get();
        inputs.leftFrontPosition = this.leftEncoderFront.getPosition();
        inputs.rightFrontPosition = this.rightEncoderFront.getPosition();
        inputs.leftBackPosition = this.leftEncoderBack.getPosition();
        inputs.rightBackPosition = this.rightEncoderBack.getPosition();
        inputs.leftVelocity = this.leftEncoderFront.getVelocity();
        inputs.rightVelocity = this.rightEncoderFront.getVelocity();
        inputs.leftCurrent = this.leftMotorFront.getOutputCurrent();
        inputs.rightCurrent = this.rightMotorFront.getOutputCurrent();
        inputs.brake = this.leftMotorFront.getIdleMode() == CANSparkMax.IdleMode.kBrake;
        inputs.heading = this.pidgey.getRotation2d();
        inputs.heading3d = this.pidgey.getRotation3d();
    }

}
