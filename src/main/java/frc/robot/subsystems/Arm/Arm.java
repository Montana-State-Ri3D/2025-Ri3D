package frc.robot.subsystems.Arm;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase{
    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
    public Arm(ArmIO io){
        this.io=io;
        io.updateInputs(inputs);
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        io.updatePIDValues();
        Logger.processInputs("Arm", inputs);
        Logger.recordOutput("Arm/CurrentCommand",
                this.getCurrentCommand() != null ? this.getCurrentCommand().getName() : "none");
    }

    public void setElevatorPos(double pos) {
        io.setElevatorPos(pos);
    }

    public void setElbowPos(double elbowAngle) {
        io.setElbowPos(elbowAngle);
    }

    public void setWristPos(double wristAngle) {
        io.setWristPos(wristAngle);
    }

    public void setArmPosition(ArmPosition position) {
        setElevatorPos(position.elevatorPosition);
        setElbowPos(position.elbowPosition);
        setWristPos(position.wristPosition);
    }

    public ArmPosition getPosition() {
        return new ArmPosition(inputs.elevatorLeaderPosition, inputs.elbowPosition, inputs.wristPosition);
    }

    public void setElevatorPower(double power){
        io.setElevatorPower(power);
    }

    public void setElbowPower(double power){
        io.setElbowPower(power);
    }

    public void setWristPower(double power){
        io.setWristPower(power);
    }

    public boolean limitSwitchHit() {
        return inputs.limitSwitchHit;
    }
 
    public void setElevatorLimits() {
        io.setElevatorLimits();
    }

    public void stop() {
        io.stop();
    }

    // immutable arm position class
    public static class ArmPosition {
        public final double elevatorPosition;
        public final double elbowPosition;
        public final double wristPosition;

        public ArmPosition(double elevatorPosition, double elbowPosition, double wristPosition) {
            this.elevatorPosition = elevatorPosition;
            this.elbowPosition = elbowPosition;
            this.wristPosition = wristPosition;
        }
    }
}