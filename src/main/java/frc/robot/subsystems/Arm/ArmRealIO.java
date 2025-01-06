package frc.robot.subsystems.Arm;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;

import frc.robot.Constants.ArmConstants;
import frc.robot.utilities.TunablePidValues;
import frc.robot.utilities.TunablePidValues.PIDValues;

public class ArmRealIO implements ArmIO {

    private double elevatorTargetPosition;
    private double elbowTargetPosition;
    private double wristTargetPosition;

    private SparkPIDController elevatorPIDController;
    private SparkPIDController elbowPIDController;
    private SparkPIDController wristPIDController;

    private CANSparkMax elevatorLeader;
    private CANSparkMax elevatorFollower;
    // private CANSparkMax elbowLeader;
    // private CANSparkMax elbowFollower;
    private CANSparkMax wrist;
    
    private RelativeEncoder elevatorLeaderEncoder;
    private RelativeEncoder elevatorFollowerEncoder;
    private RelativeEncoder elbowLeaderEncoder;
    private RelativeEncoder elbowFollowerEncoder;
    private SparkAbsoluteEncoder elbowEncoder;
    private SparkAbsoluteEncoder wristEncoder;

    private CANSparkMax[] motors;
    private RelativeEncoder[] encoders;

    private TunablePidValues elevatorTunablePid;
    private TunablePidValues elbowTunablePid;
    private TunablePidValues wristTunablePid;

    private DigitalInput limitSwitch;

    public ArmRealIO(int elevatorLeader, int elevatorFollower, int elbowLeader, int elbowFollower, int wrist, int limitSwitch) {
        this.elevatorLeader = new CANSparkMax(elevatorLeader, CANSparkMax.MotorType.kBrushless);
        this.elevatorFollower = new CANSparkMax(elevatorFollower, CANSparkMax.MotorType.kBrushless);
        // this.elbowLeader = new CANSparkMax(elbowLeader, CANSparkMax.MotorType.kBrushless);
        // this.elbowFollower = new CANSparkMax(elbowFollower, CANSparkMax.MotorType.kBrushless);
        // this.wrist = new CANSparkMax(wrist, CANSparkMax.MotorType.kBrushless);

        this.motors = new CANSparkMax[2];
        this.motors[0] = this.elevatorLeader;
        this.motors[1] = this.elevatorFollower;
        // this.motors[2] = this.elbowLeader;
        // this.motors[3] = this.elbowFollower;
        // this.motors[4] = this.wrist;

        this.limitSwitch = new DigitalInput(limitSwitch);

        for (CANSparkMax motor : this.motors) {
            motor.restoreFactoryDefaults();
            motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        }

        this.elevatorLeader.setSmartCurrentLimit(80);
        this.elevatorFollower.setSmartCurrentLimit(80);

        this.elevatorLeader.setInverted(false);
        // this.elbowFollower.setInverted(false);
        // this.wrist.setInverted(false);

        this.elevatorFollower.follow(this.elevatorLeader, true);
        // this.elbowFollower.follow(this.elbowLeader, false);

        // encoder init
        this.elevatorLeaderEncoder = this.elevatorLeader.getEncoder();
        this.elevatorFollowerEncoder = this.elevatorFollower.getEncoder();


        // this.elbowEncoder = this.elbowLeader.getAbsoluteEncoder(Type.kDutyCycle);
        // this.elbowLeaderEncoder = this.elbowLeader.getEncoder();
        // this.elbowFollowerEncoder = this.elbowFollower.getEncoder();
        // this.wristEncoder = this.wrist.getAbsoluteEncoder(Type.kDutyCycle);

        // 4.0 instead of 2.0 to account for cascade rigging of elevator
        elevatorLeaderEncoder.setPositionConversionFactor(ArmConstants.ELEVATOR_SPROCKET_RADIUS * 4.0 * Math.PI * ArmConstants.ELEVATOR_RATIO );
        elevatorFollowerEncoder.setPositionConversionFactor(ArmConstants.ELEVATOR_SPROCKET_RADIUS * 4.0 * Math.PI * ArmConstants.ELEVATOR_RATIO);
        
        elevatorLeaderEncoder.setVelocityConversionFactor(ArmConstants.ELEVATOR_SPROCKET_RADIUS * 4.0 * Math.PI * ArmConstants.ELEVATOR_RATIO / 60.0);
        elevatorFollowerEncoder.setVelocityConversionFactor(ArmConstants.ELEVATOR_SPROCKET_RADIUS * 4.0 * Math.PI * ArmConstants.ELEVATOR_RATIO / 60.0);

        // elbowEncoder.setPositionConversionFactor(Math.PI*2);
        // elbowEncoder.setVelocityConversionFactor(Math.PI*2/60);
        // wristEncoder.setPositionConversionFactor(Math.PI*2);
        // wristEncoder.setVelocityConversionFactor(Math.PI*2/60);

        // this.elevatorLeader.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 0);
        // this.elevatorLeader.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);

        // this.elevatorLeader.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, ArmConstants.ELEVATOR_HEIGHT);
        // this.elevatorLeader.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

        this.encoders = new RelativeEncoder[2];
        this.encoders[0] = this.elevatorLeaderEncoder;
        this.encoders[1] = this.elevatorFollowerEncoder;

        // elevator PID
        elevatorTunablePid = new TunablePidValues(
            "Elevator PID",
            new PIDValues(
                ArmConstants.ELEVATOR_kP, 
                ArmConstants.ELEVATOR_kI, 
                ArmConstants.ELEVATOR_kD, 
                0
            )
        );
        PIDValues elevatorPidValues = elevatorTunablePid.get();

        elevatorPIDController = this.elevatorLeader.getPIDController();

        elevatorPIDController.setP(elevatorPidValues.kP);
        elevatorPIDController.setI(elevatorPidValues.kI);
        elevatorPIDController.setD(elevatorPidValues.kD);

        elevatorPIDController.setFeedbackDevice(elevatorLeaderEncoder);
        elevatorPIDController.setOutputRange(-1.0, 1.0);

        // elbow PID
        // elbowTunablePid = new TunablePidValues(
        //     "Elbow PID",
        //     new PIDValues(
        //         ArmConstants.ELBOW_kP, 
        //         ArmConstants.ELBOW_kI, 
        //         ArmConstants.ELBOW_kD, 
        //         0
        //     )
        // );
        // PIDValues elbowPidValues = elbowTunablePid.get();

        // elbowPIDController = this.elbowLeader.getPIDController();

        // elbowPIDController.setP(elbowPidValues.kP);
        // elbowPIDController.setI(elbowPidValues.kI);
        // elbowPIDController.setD(elbowPidValues.kD);

        // elbowPIDController.setFeedbackDevice(elbowEncoder);
        // elbowPIDController.setOutputRange(-1.00, 1.00);

        // // wrist PID
        // wristTunablePid = new TunablePidValues(
        //     "Wrist PID",
        //     new PIDValues(
        //         ArmConstants.WRIST_kP, 
        //         ArmConstants.WRIST_kI, 
        //         ArmConstants.WRIST_kD, 
        //         0
        //     )
        // );
        // PIDValues wristPidValues = wristTunablePid.get();

        // wristPIDController = this.wrist.getPIDController();

        // wristPIDController.setP(wristPidValues.kP);
        // wristPIDController.setI(wristPidValues.kI);
        // wristPIDController.setD(wristPidValues.kD);

        // wristPIDController.setFeedbackDevice(wristEncoder);
        // wristPIDController.setOutputRange(-1.00, 1.00);
    }

    public void updatePIDValues() {
        PIDValues elevatorPidValues = elevatorTunablePid.get();
        // PIDValues elbowPidValues = elbowTunablePid.get();
        // PIDValues wristPidValues = wristTunablePid.get();

        if (elevatorPidValues.updated) {
            elevatorPIDController.setP(elevatorPidValues.kP);
            elevatorPIDController.setI(elevatorPidValues.kI);
            elevatorPIDController.setD(elevatorPidValues.kD);
        }

        // if (elbowPidValues.updated) {
        //     elbowPIDController.setP(elbowPidValues.kP);
        //     elbowPIDController.setI(elbowPidValues.kI);
        //     elbowPIDController.setD(elbowPidValues.kD);
        // }

        // if (wristPidValues.updated) {
        //     wristPIDController.setP(wristPidValues.kP);
        //     wristPIDController.setI(wristPidValues.kI);
        //     wristPIDController.setD(wristPidValues.kD);
        // }
    }

    public void setElevatorPower(double power){
        this.elevatorLeader.set(power);
    }

    public void stop() {
        for (CANSparkMax motor : this.motors) {
            motor.stopMotor();
        }
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

    // TODO: Figure out if forward is to make it go down, or if it should be changed to reverse
    // ensure that the elevator is at the desired lower limit position before calling
    public void setElevatorLimits() {
        this.elevatorLeaderEncoder.setPosition(0);
        this.elevatorFollowerEncoder.setPosition(0);
    }

    public void setElevatorPos(double elevatorPos) {
        elevatorTargetPosition = elevatorPos;
        elevatorPIDController.setReference(elevatorPos, ControlType.kPosition);
    }

    // public void setElbowPos(double elbowPos) {
    //     elbowTargetPosition = elbowPos;
    //     elbowPIDController.setReference(elbowPos, ControlType.kPosition);
    // }

    // public void setWristPos(double wristPos) {
    //     wristTargetPosition = wristPos;
    //     wristPIDController.setReference(wristPos, ControlType.kPosition);
    // }

    public void updateInputs(ArmIOInputs inputs) {
        inputs.elevatorLeaderPower = this.elevatorLeader.get();
        inputs.elevatorFollowerPower = this.elevatorFollower.get();
        // inputs.elbowLeaderPower = this.elbowLeader.get();
        // inputs.elbowFollowerPower = this.elbowFollower.get();
        // inputs.wristPower = this.wrist.get();

        inputs.elevatorLeaderPosition = this.elevatorLeaderEncoder.getPosition();
        inputs.elevatorFollowerPosition = this.elevatorFollowerEncoder.getPosition();
        // inputs.elbowPosition = this.elbowEncoder.getPosition();
        // inputs.elbowLeaderPosition = this.elbowLeaderEncoder.getPosition();
        // inputs.elbowFollowerPosition = this.elbowFollowerEncoder.getPosition();
        // inputs.wristPosition = this.wristEncoder.getPosition();

        inputs.elevatorTargetPosition = elevatorTargetPosition;
        // inputs.elbowTargetPosition = elbowTargetPosition;
        // inputs.wristTargetPosition = wristTargetPosition;

        inputs.elevatorLeaderVelocity = this.elevatorLeaderEncoder.getVelocity();
        inputs.elevatorFollowerVelocity = this.elevatorFollowerEncoder.getVelocity();
        // inputs.elbowVelocity = this.elbowEncoder.getVelocity();
        // inputs.elbowLeaderVelocity = this.elbowLeaderEncoder.getVelocity();
        // inputs.elbowFollowerVelocity = this.elbowFollowerEncoder.getVelocity();
        // inputs.wristVelocity = this.wristEncoder.getVelocity();

        inputs.elevatorLeaderCurrent = this.elevatorLeader.getOutputCurrent();
        inputs.elevatorFollowerCurrent = this.elevatorFollower.getOutputCurrent();
        // inputs.elbowLeaderCurrent = this.elbowLeader.getOutputCurrent();
        // inputs.elbowFollowerCurrent = this.elbowFollower.getOutputCurrent();
        // inputs.wristCurrent = this.wrist.getOutputCurrent();

        inputs.isBrake = this.elevatorLeader.getIdleMode() == CANSparkMax.IdleMode.kBrake;

        inputs.limitSwitchHit = limitSwitch.get();
    }
}