package frc.robot.subsystems.Arm;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import frc.robot.Constants.ArmConstants;

public class ArmRealIO implements ArmIO {

    private double elevatorTargetPosition;
    private double elbowTargetPosition;
    private double wristTargetPosition;

    private SparkPIDController elevatorPIDController;

    private CANSparkMax elevatorLeader;
    private CANSparkMax elevatorFollower;
    
    private RelativeEncoder elevatorLeaderEncoder;
    private RelativeEncoder elevatorFollowerEncoder;

    private CANSparkMax[] motors;
    private RelativeEncoder[] encoders;

    public ArmRealIO(int elevatorLeader, int elevatorFollower){
        this.elevatorLeader = new CANSparkMax(elevatorLeader, CANSparkMax.MotorType.kBrushless);
        this.elevatorFollower = new CANSparkMax(elevatorFollower, CANSparkMax.MotorType.kBrushless);

        this.motors = new CANSparkMax[2];
        this.motors[0] = this.elevatorLeader;
        this.motors[1] = this.elevatorFollower;


        for (CANSparkMax motor : this.motors) {
            motor.restoreFactoryDefaults();
            motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
            motor.setSmartCurrentLimit(80);
        }

        this.elevatorLeader.setInverted(false);

        this.elevatorFollower.follow(this.elevatorLeader, true);

        this.elevatorLeaderEncoder = this.elevatorLeader.getEncoder();
        this.elevatorFollowerEncoder = this.elevatorFollower.getEncoder();

        this.encoders = new RelativeEncoder[2];
        this.encoders[0] = this.elevatorLeaderEncoder;
        this.encoders[1] = this.elevatorFollowerEncoder;

        elevatorPIDController = this.elevatorLeader.getPIDController();

        elevatorPIDController.setP(ArmConstants.ELEVATOR_kP);
        elevatorPIDController.setI(ArmConstants.ELEVATOR_kI);
        elevatorPIDController.setD(ArmConstants.ELEVATOR_kD);

        elevatorPIDController.setFeedbackDevice(elevatorLeaderEncoder);
        elevatorPIDController.setOutputRange(-1.00, 1.00);
    }

    public void setElevatorPower(double power){
        this.elevatorLeader.set(power);
    }
    
    public void toggleMode() {
        if (this.elevatorLeader.getIdleMode() == CANSparkMax.IdleMode.kBrake) {
            for (CANSparkMax motor : this.motors) {
                motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
            }
        } else {
            for (CANSparkMax motor : this.motors) {
                motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
            }
        }
    }

    public void setElevatorPose(double elevatorPos) {
        elevatorTargetPosition = elevatorPos;
        elevatorPIDController.setReference(elevatorPos, ControlType.kPosition);
    }

    public void setElbowPose(double elbowPos) {
        elbowTargetPosition = elbowPos;
        // elbowPIDController.setReference(elbowPos, ControlType.kPosition);
    }

    public void setWristPose(double wristPos) {
        wristTargetPosition = wristPos;
        // wristPIDController.setReference(wristPos, ControlType.kPosition);
    }

    public void updateInputs(ArmIOInputs inputs) {
        inputs.elevatorLeaderPower = this.elevatorLeader.get();
        inputs.elevatorFollowerPower = this.elevatorFollower.get();
        inputs.elevatorLeaderPosition = this.elevatorLeaderEncoder.getPosition();
        inputs.elevatorFollowerPosition = this.elevatorFollowerEncoder.getPosition();
        inputs.elevatorTargetPosition = elevatorTargetPosition;
        inputs.elevatorLeaderVelocity = this.elevatorLeaderEncoder.getVelocity();
        inputs.elevatorFollowerVelocity = this.elevatorFollowerEncoder.getVelocity();
        inputs.elevatorLeaderCurrent = this.elevatorLeader.getOutputCurrent();
        inputs.elevatorFollowerCurrent = this.elevatorFollower.getOutputCurrent();
        inputs.isBrake = this.elevatorLeader.getIdleMode() == CANSparkMax.IdleMode.kBrake;
    }
}