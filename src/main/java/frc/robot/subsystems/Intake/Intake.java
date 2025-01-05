package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

//  11-1 gearbox for the wheels, 15-1 gearbox for the active pivot, also uses mini-neos


public class Intake extends SubsystemBase{
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    public Intake(IntakeIO io){
        this.io=io;
        io.updateInputs(inputs);

    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
        Logger.recordOutput("Intake/CurrentCommand",
                this.getCurrentCommand() != null ? this.getCurrentCommand().getName() : "none");
    }

    public void setPower(double leftPower, double rightPower){
        io.setPower(leftPower, rightPower);
    }

    public void setPivotAngle(double angle) {
        io.setAngle(angle);
    }

    public double getLeftCurrent() {
        return inputs.leftCurrent;
    }

    public double getRightCurrent() {
        return inputs.rightCurrent;
    }

    public double getPivotAngle() {
        return inputs.pivotAngle;
    }
}
