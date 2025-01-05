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
        Logger.processInputs("Arm", inputs);
        Logger.recordOutput("Arm/CurrentCommand",
                this.getCurrentCommand() != null ? this.getCurrentCommand().getName() : "none");
    }

    public void setElevatorPos(double pos) {
        io.setElevatorPos(pos);
    }

    public void setElbowPos(double elbowAngle) {
        io.setElevatorPos(elbowAngle);
    }

    public void setWristPos(double wristAngle) {
        io.setElevatorPos(wristAngle);
    }
}