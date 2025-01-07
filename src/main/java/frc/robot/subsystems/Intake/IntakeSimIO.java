package frc.robot.subsystems.Intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import frc.robot.Constants.IntakeConstants;




public class IntakeSimIO implements IntakeIO {
    private double targetAngle;

    private boolean isBrake;

    private DCMotorSim leftIntakeSim;
    private DCMotorSim rightIntakeSim;
    private Encoder leftEncoder;
    private Encoder rightEncoder;
    private Encoder pivotEncoder;
    private EncoderSim leftEncoderSim;
    private EncoderSim rightEncoderSim;
    private EncoderSim pivotEncoderSim;

    public IntakeSimIO(){
        isBrake = true;

        leftIntakeSim = new DCMotorSim(DCMotor.getNEO(1), IntakeConstants.INTAKE_RATIO, 1);
        rightIntakeSim = new DCMotorSim(DCMotor.getNEO(1), IntakeConstants.INTAKE_RATIO, 1);
        leftEncoder = new Encoder(4, 5);
        rightEncoder = new Encoder(6, 7);
        pivotEncoder = new Encoder(8, 9);
        leftEncoderSim = new EncoderSim(leftEncoder);
        rightEncoderSim = new EncoderSim(rightEncoder);
        pivotEncoderSim = new EncoderSim(pivotEncoder);

    }
    
    
    public void setBrake(boolean isBrake) {
       this.isBrake = isBrake;
       
    }


    public void setPower(double leftPower, double rightPower) {
       leftIntakeSim.setInputVoltage(leftPower*12);
       rightIntakeSim.setInputVoltage(rightPower*12);
    }

    public void setAngle(double angle) {
        targetAngle = angle;
    }


    public void updateInputs(IntakeIOInputs inputs) {
        inputs.isBrake = isBrake;
        inputs.leftCurrent = leftIntakeSim.getCurrentDrawAmps();
        inputs.rightCurrent = rightIntakeSim.getCurrentDrawAmps();
        inputs.rightVelo = rightIntakeSim.getAngularVelocityRadPerSec();
        inputs.leftVelo = leftIntakeSim.getAngularVelocityRadPerSec();
        inputs.targetAngle = targetAngle;
    }
}