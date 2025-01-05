package frc.robot.subsystems.Arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class ArmRealIO implements ArmIO{

    private CANSparkMax elevatorMaster;
    private CANSparkMax elevatorSlave;
    
    private RelativeEncoder elevatorMasterEncoder;
    private RelativeEncoder elevatorSlaveEncoder;

    private CANSparkMax[] motors;
    private RelativeEncoder[] encoders;

    public ArmRealIO(int elevatorMaster, int elevatorSlave){
        this.elevatorMaster = new CANSparkMax(elevatorMaster, CANSparkMax.MotorType.kBrushless);
        this.elevatorSlave = new CANSparkMax(elevatorSlave, CANSparkMax.MotorType.kBrushless);

        this.motors = new CANSparkMax[2];
        this.motors[0] = this.elevatorMaster;
        this.motors[1] = this.elevatorSlave;


        for (CANSparkMax motor : this.motors) {
            motor.restoreFactoryDefaults();
            motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
            motor.setSmartCurrentLimit(40);
        }

        this.elevatorMaster.setInverted(false);

        this.elevatorSlave.follow(this.elevatorMaster, true);

        this.elevatorMasterEncoder = this.elevatorMaster.getEncoder();
        this.elevatorSlaveEncoder = this.elevatorSlave.getEncoder();


        this.encoders = new RelativeEncoder[2];
        this.encoders[0] = this.elevatorMasterEncoder;
        this.encoders[1] = this.elevatorSlaveEncoder;
    }

    public void setElevatorPower(double power){
        this.elevatorMaster.set(power);
    }
    
    public void toggleMode() {
        if (this.elevatorMaster.getIdleMode() == CANSparkMax.IdleMode.kBrake) {
            for (CANSparkMax motor : this.motors) {
                motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
            }
        } else {
            for (CANSparkMax motor : this.motors) {
                motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
            }
        }
    }

    public void updateInputs(ArmIOInputs inputs) {
        inputs.elevatorMasterPower = this.elevatorMaster.get();
        inputs.elevatorSlavePower = this.elevatorSlave.get();
        inputs.elevatorMasterPosition = this.elevatorMasterEncoder.getPosition();
        inputs.elevatorSlavePosition = this.elevatorSlaveEncoder.getPosition();
        inputs.elevatorMasterVelocity = this.elevatorMasterEncoder.getVelocity();
        inputs.elevatorSlaveVelocity = this.elevatorSlaveEncoder.getVelocity();
        inputs.elevatorMasterCurrent = this.elevatorMaster.getOutputCurrent();
        inputs.elevatorSlaveCurrent = this.elevatorSlave.getOutputCurrent();
        inputs.isBrake = this.elevatorMaster.getIdleMode() == CANSparkMax.IdleMode.kBrake;
    }
}