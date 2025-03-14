package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//  11-1 gearbox for the wheels, 15-1 gearbox for the active pivot, also uses mini-neos


public class Intake extends SubsystemBase{
    public enum IntakePosition {
        CORAL(Units.degreesToRadians(-107.0)),
        ALGAE(Units.degreesToRadians(-100.0)),
        HANDOFF(Units.degreesToRadians(-2.0)),
        PROCESSOR(0.0),
        IDLE(Units.degreesToRadians(-5.0));
        
        private double position;
    
        private IntakePosition(double position) {
          this.position = position; 
        }
        public double getPosition() {
          return position;
        }
    }
    IntakePosition position;

    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    public Intake(IntakeIO io){
        this.io=io;
        inputs.hasAlgae = false;
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

    public void setPivotPower(double power) {
        io.setPivotPower(power);
    }

    public void setPivotPosition(IntakePosition position) {
        this.position = position;
        io.setAngle(position.getPosition());
    }

    public void setPivotAngle(double angle) {
        io.setAngle(angle);
    }

    public void stop() {
        io.stop();
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

    public boolean hasObject() {
        return inputs.hasCoral || inputs.hasAlgae;
    }

    public void pickupAlgae() {
        inputs.hasAlgae = true;
    }

    public void ejectObject() {
        inputs.hasAlgae = false;
    }
}
