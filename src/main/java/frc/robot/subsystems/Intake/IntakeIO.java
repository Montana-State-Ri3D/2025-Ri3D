package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;


public interface IntakeIO {
    
    @AutoLog
    public class IntakeIOInputs{
        public double rightPower;

        public double leftPower;

        public double rightCurrent;

        public double leftCurrent;

        public boolean isBrake;

        public double leftVelo;

        public double rightVelo;
    }
    
    default void updateInputs(IntakeIOInputs inputs){}
    default void setPower(double leftPower, double rightPower){}
    default void setBrake(boolean isBrake){}
    
}