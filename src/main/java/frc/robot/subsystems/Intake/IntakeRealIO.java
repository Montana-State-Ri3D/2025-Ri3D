package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import edu.wpi.first.wpilibj.DigitalInput;

import frc.robot.Constants.IntakeConstants;

public class IntakeRealIO implements IntakeIO{    
    private boolean isBrake;
    private double targetAngle;

    private DigitalInput beamBreak;

    private CANSparkMax leftIntake;
    private CANSparkMax rightIntake;
    private CANSparkMax pivot;

    private SparkPIDController pivotPIDController;

    private RelativeEncoder leftIntakeEncoder;
    private RelativeEncoder rightIntakeEncoder;
    private RelativeEncoder pivotEncoder;
    //private CANcoder pivotCANcoder;
    
    private CANSparkMax[] intakeMotors;

    public IntakeRealIO(int leftIntake, int rightIntake, int pivot, int pivotCANcoder, int beamBreak){
        this.beamBreak = new DigitalInput(beamBreak);

        this.leftIntake = new CANSparkMax(leftIntake, CANSparkMax.MotorType.kBrushless);
        this.rightIntake = new CANSparkMax(rightIntake, CANSparkMax.MotorType.kBrushless);
        this.pivot = new CANSparkMax(pivot, CANSparkMax.MotorType.kBrushless);

        this.intakeMotors = new CANSparkMax[2];
        this.intakeMotors[0] = this.leftIntake;
        this.intakeMotors[1] = this.rightIntake;

        for (CANSparkMax motor: this.intakeMotors){
            motor.restoreFactoryDefaults();
            motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        }

        this.pivot.restoreFactoryDefaults();
        this.pivot.setIdleMode(CANSparkMax.IdleMode.kBrake);

        this.leftIntake.setInverted(false);
        this.rightIntake.setInverted(true);

        pivotPIDController = this.pivot.getPIDController();

        leftIntakeEncoder = this.leftIntake.getEncoder();
        rightIntakeEncoder = this.rightIntake.getEncoder();
        pivotEncoder = this.pivot.getEncoder();
        //this.pivotCANcoder = new CANcoder(pivotCANcoder);

        pivotEncoder.setPositionConversionFactor(Math.PI*2*IntakeConstants.INTAKE_RATIO);
        pivotEncoder.setVelocityConversionFactor(Math.PI*2*IntakeConstants.INTAKE_RATIO/60);

        //pivotEncoder.setPosition(this.pivotCANcoder.getPosition().getValue());
        pivotEncoder.setPosition(0.0);

        pivotPIDController.setP(IntakeConstants.PIVOT_kP);
        pivotPIDController.setI(IntakeConstants.PIVOT_kI);
        pivotPIDController.setD(IntakeConstants.PIVOT_kD);
        pivotPIDController.setFeedbackDevice(pivotEncoder);
        pivotPIDController.setOutputRange(-0.40, 0.40);

        rightIntakeEncoder.setPositionConversionFactor(IntakeConstants.INTAKE_WHEEL_RADIUS*2.0*Math.PI*IntakeConstants.INTAKE_RATIO);
        leftIntakeEncoder.setPositionConversionFactor(IntakeConstants.INTAKE_WHEEL_RADIUS*2.0*Math.PI*IntakeConstants.INTAKE_RATIO);

        // rightIntakeEncoder.setVelocityConversionFactor(IntakeConstants.INTAKE_WHEEL_RADIUS*2.0*Math.PI*IntakeConstants.INTAKE_RATIO/60.0);
        // leftIntakeEncoder.setVelocityConversionFactor(IntakeConstants.INTAKE_WHEEL_RADIUS*2.0*Math.PI*IntakeConstants.INTAKE_RATIO/60.0);        
    }

    public void setBrake(boolean isBrake) {
        if(isBrake){
            for (CANSparkMax motor: this.intakeMotors){
                motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
            }
        }else{
            for (CANSparkMax motor: this.intakeMotors){
                motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
            }
        }
        this.isBrake = isBrake;
    }

    public void setPower(double leftPower, double rightPower) {
       leftIntake.set(leftPower);
       rightIntake.set(rightPower);
    }

    public void setPivotPower(double power) {
        pivot.set(power);
    }

    public void setAngle(double angle) {
        targetAngle = angle;
        pivotPIDController.setReference(angle, ControlType.kPosition);
    }

    public void stop() {
        setPower(0.0, 0.0);
        setAngle(pivotEncoder.getPosition());
    }

    public void updateInputs(IntakeIOInputs inputs) {
        inputs.isBrake = isBrake;
        inputs.leftCurrent = leftIntake.getOutputCurrent();
        inputs.rightCurrent = rightIntake.getOutputCurrent();
        inputs.pivotCurrent = pivot.getOutputCurrent();
        inputs.rightVelo = rightIntakeEncoder.getVelocity();
        inputs.leftVelo = leftIntakeEncoder.getVelocity();
        inputs.pivotVelo = pivotEncoder.getVelocity();
        inputs.leftPower = leftIntake.getAppliedOutput();
        inputs.rightPower = rightIntake.getAppliedOutput();
        inputs.pivotPower = pivot.getAppliedOutput();
        inputs.pivotAngle = pivotEncoder.getPosition();
        inputs.targetAngle = targetAngle;
        inputs.hasCoral = !beamBreak.get();
        inputs.hasAlgae = false;  
    }
}