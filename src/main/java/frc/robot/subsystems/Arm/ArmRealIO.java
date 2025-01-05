package frc.robot.subsystems.Arm;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;

import frc.robot.Constants.ArmConstants;

public class ArmRealIO implements ArmIO {

    private double elevatorTargetPosition;
    private double elbowTargetPosition;
    private double wristTargetPosition;

    private SparkPIDController elevatorPIDController;
    private SparkPIDController elbowPIDController;
    private SparkPIDController wristPIDController;

    private CANSparkMax elevatorLeader;
    private CANSparkMax elevatorFollower;
    private CANSparkMax elbowLeader;
    private CANSparkMax elbowFollower;
    private CANSparkMax wrist;
    
    private RelativeEncoder elevatorLeaderEncoder;
    private RelativeEncoder elevatorFollowerEncoder;
    private SparkAbsoluteEncoder elbowEncoder;
    private SparkAbsoluteEncoder wristEncoder;

    private CANSparkMax[] motors;
    private RelativeEncoder[] encoders;

    public ArmRealIO(int elevatorLeader, int elevatorFollower, int elbowLeader, int elbowFollower, int wrist){
        this.elevatorLeader = new CANSparkMax(elevatorLeader, CANSparkMax.MotorType.kBrushless);
        this.elevatorFollower = new CANSparkMax(elevatorFollower, CANSparkMax.MotorType.kBrushless);
        this.elbowLeader = new CANSparkMax(elbowLeader, CANSparkMax.MotorType.kBrushless);
        this.elbowFollower = new CANSparkMax(elbowFollower, CANSparkMax.MotorType.kBrushless);
        this.wrist = new CANSparkMax(wrist, CANSparkMax.MotorType.kBrushless);

        this.motors = new CANSparkMax[5];
        this.motors[0] = this.elevatorLeader;
        this.motors[1] = this.elevatorFollower;
        this.motors[2] = this.elbowLeader;
        this.motors[3] = this.elbowFollower;
        this.motors[4] = this.wrist;


        for (CANSparkMax motor : this.motors) {
            motor.restoreFactoryDefaults();
            motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        }

        this.elevatorLeader.setSmartCurrentLimit(80);
        this.elevatorFollower.setSmartCurrentLimit(80);

        this.elevatorLeader.setInverted(false);
        this.elbowFollower.setInverted(false);
        this.wrist.setInverted(false);

        this.elevatorFollower.follow(this.elevatorLeader, true);
        this.elbowFollower.follow(this.elbowLeader, false);

        // encoder init
        this.elevatorLeaderEncoder = this.elevatorLeader.getEncoder();
        this.elevatorFollowerEncoder = this.elevatorFollower.getEncoder();
        this.elbowEncoder = this.elbowLeader.getAbsoluteEncoder(Type.kDutyCycle);
        this.wristEncoder = this.wrist.getAbsoluteEncoder(Type.kDutyCycle);

        elbowEncoder.setPositionConversionFactor(Math.PI*2);
        elbowEncoder.setVelocityConversionFactor(Math.PI*2/60);
        wristEncoder.setPositionConversionFactor(Math.PI*2);
        wristEncoder.setVelocityConversionFactor(Math.PI*2/60);

        this.encoders = new RelativeEncoder[2];
        this.encoders[0] = this.elevatorLeaderEncoder;
        this.encoders[1] = this.elevatorFollowerEncoder;

        // elevator PID
        elevatorPIDController = this.elevatorLeader.getPIDController();

        elevatorPIDController.setP(ArmConstants.ELEVATOR_kP);
        elevatorPIDController.setI(ArmConstants.ELEVATOR_kI);
        elevatorPIDController.setD(ArmConstants.ELEVATOR_kD);

        elevatorPIDController.setFeedbackDevice(elevatorLeaderEncoder);
        elevatorPIDController.setOutputRange(-1.00, 1.00);

        // elbow PID
        elbowPIDController = this.elbowLeader.getPIDController();

        elbowPIDController.setP(ArmConstants.ELEVATOR_kP);
        elbowPIDController.setI(ArmConstants.ELEVATOR_kI);
        elbowPIDController.setD(ArmConstants.ELEVATOR_kD);

        elbowPIDController.setFeedbackDevice(elbowEncoder);
        elbowPIDController.setOutputRange(-1.00, 1.00);

        // wrist PID
        wristPIDController = this.wrist.getPIDController();

        wristPIDController.setP(ArmConstants.ELEVATOR_kP);
        wristPIDController.setI(ArmConstants.ELEVATOR_kI);
        wristPIDController.setD(ArmConstants.ELEVATOR_kD);

        wristPIDController.setFeedbackDevice(wristEncoder);
        wristPIDController.setOutputRange(-1.00, 1.00);
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

    public void setElevatorPos(double elevatorPos) {
        elevatorTargetPosition = elevatorPos;
        elevatorPIDController.setReference(elevatorPos, ControlType.kPosition);
    }

    public void setElbowPos(double elbowPos) {
        elbowTargetPosition = elbowPos;
        elbowPIDController.setReference(elbowPos, ControlType.kPosition);
    }

    public void setWristPos(double wristPos) {
        wristTargetPosition = wristPos;
        wristPIDController.setReference(wristPos, ControlType.kPosition);
    }

    public void updateInputs(ArmIOInputs inputs) {
        inputs.elevatorLeaderPower = this.elevatorLeader.get();
        inputs.elevatorFollowerPower = this.elevatorFollower.get();
        inputs.elbowLeaderPower = this.elbowLeader.get();
        inputs.elbowFollowerPower = this.elbowFollower.get();
        inputs.wristPower = this.wrist.get();

        inputs.elevatorLeaderPosition = this.elevatorLeaderEncoder.getPosition();
        inputs.elevatorFollowerPosition = this.elevatorFollowerEncoder.getPosition();
        inputs.elbowPosition = this.elbowEncoder.getPosition();
        inputs.wristPosition = this.wristEncoder.getPosition();

        inputs.elevatorTargetPosition = elevatorTargetPosition;
        inputs.elbowTargetPosition = elbowTargetPosition;
        inputs.wristTargetPosition = wristTargetPosition;

        inputs.elevatorLeaderVelocity = this.elevatorLeaderEncoder.getVelocity();
        inputs.elevatorFollowerVelocity = this.elevatorFollowerEncoder.getVelocity();
        inputs.elbowVelocity = this.elbowEncoder.getVelocity();
        inputs.wristVelocity = this.wristEncoder.getVelocity();

        inputs.elevatorLeaderCurrent = this.elevatorLeader.getOutputCurrent();
        inputs.elevatorFollowerCurrent = this.elevatorFollower.getOutputCurrent();
        inputs.elbowLeaderCurrent = this.elbowLeader.getOutputCurrent();
        inputs.elbowFollowerCurrent = this.elbowFollower.getOutputCurrent();
        inputs.wristCurrent = this.wrist.getOutputCurrent();

        inputs.isBrake = this.elevatorLeader.getIdleMode() == CANSparkMax.IdleMode.kBrake;
    }
}