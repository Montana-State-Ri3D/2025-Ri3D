package frc.robot.subsystems.DriveTrain;

import frc.robot.Constants.DriveTrainConstants;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;

/** Add your docs here. */
public class DriveTrainSimIO implements DriveTrainIO {

    private boolean isBrake;

    private Encoder leftEncoder;
    private Encoder rightEncoder;

    private EncoderSim leftEncoderSim;
    private EncoderSim rightEncoderSim;

    private AnalogGyro gyro;
    private AnalogGyroSim gyroSim;

    private DifferentialDrivetrainSim driveSim;

    private final int NEO_TPR = 42;

    private PWMSparkMax leftMotorFront;
    private PWMSparkMax leftMotorBack;
    private PWMSparkMax rightMotorFront;
    private PWMSparkMax rightMotorBack;

    private double batteryMoi = 12.5 / 2.2 * Math.pow(Units.inchesToMeters(10), 2);
    private double gearboxMoi = (2.8 /* CIM motor */ * 2 / 2.2 + 2.0 /* Toughbox Mini- ish */)
            * Math.pow(Units.inchesToMeters(26.0 / 2.0), 2);

    public DriveTrainSimIO() {
        leftEncoder = new Encoder(0, 1);
        rightEncoder = new Encoder(2, 3);

        leftEncoderSim = new EncoderSim(rightEncoder);
        rightEncoderSim = new EncoderSim(leftEncoder);

        leftEncoder.setDistancePerPulse(2 * Math.PI * DriveTrainConstants.DRIVE_WHEEL_RADIUS / NEO_TPR);
        rightEncoder.setDistancePerPulse(2 * Math.PI * DriveTrainConstants.DRIVE_WHEEL_RADIUS / NEO_TPR);

        gyro = new AnalogGyro(1);
        gyroSim = new AnalogGyroSim(gyro);

        driveSim = new DifferentialDrivetrainSim(
                DCMotor.getNEO(2),
                DriveTrainConstants.DRIVE_RADIO,
                this.batteryMoi + this.gearboxMoi,
                Units.lbsToKilograms(150),
                Units.inchesToMeters(DriveTrainConstants.DRIVE_WHEEL_RADIUS),
                Units.inchesToMeters(32.0),

                // The standard deviations for measurement noise:
                // x and y: 0.001 m
                // heading: 0.001 rad
                // l and r velocity: 0.1 m/s
                // l and r position: 0.005 m
                VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));

        leftMotorFront = new PWMSparkMax(0);
        leftMotorBack = new PWMSparkMax(1);
        rightMotorFront = new PWMSparkMax(2);
        rightMotorBack = new PWMSparkMax(3);

        isBrake = false;
    }

    public void updateInputs(DriveTrainIOInputs inputs) {

        driveSim.setInputs(-leftMotorFront.get() * RobotController.getInputVoltage(),
                -rightMotorFront.get() * RobotController.getInputVoltage());

        driveSim.update(0.02);

        // Update all of our sensors.
        leftEncoderSim.setDistance(-driveSim.getLeftPositionMeters());
        leftEncoderSim.setRate(-driveSim.getLeftVelocityMetersPerSecond());

        rightEncoderSim.setDistance(-driveSim.getRightPositionMeters());
        rightEncoderSim.setRate(-driveSim.getRightVelocityMetersPerSecond());

        gyroSim.setAngle(driveSim.getHeading().getDegrees());

        inputs.brake = isBrake;
        inputs.leftCurrent = 0;
        inputs.rightCurrent = 0;
        inputs.leftPower = leftMotorFront.get();
        inputs.rightPower = rightMotorFront.get();
        inputs.leftPosition = leftEncoder.getDistance();
        inputs.rightPosition = rightEncoder.getDistance();
        inputs.leftVelocity = leftEncoder.getRate();
        inputs.rightVelocity = rightEncoder.getRate();
        inputs.heading = Rotation2d.fromDegrees(-gyro.getAngle());

    }

    public void drive(double leftPower, double rightPower) {
        leftMotorFront.set(leftPower);
        leftMotorBack.set(leftPower);
        rightMotorFront.set(rightPower);
        rightMotorBack.set(rightPower);
    }

    public void toggleMode() {
        isBrake = !isBrake;
    }
}