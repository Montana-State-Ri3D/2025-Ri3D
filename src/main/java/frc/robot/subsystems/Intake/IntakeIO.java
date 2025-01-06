package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.Constants.IntakeConstants;


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

        public double pivotAngle;

        public double pivotVelo;

        public double targetAngle;

        public Intake.IntakePosition position;
    }
    
    default void updateInputs(IntakeIOInputs inputs){}
    default void setPower(double leftPower, double rightPower){}
    default void setBrake(boolean isBrake){}
    default void setAngle(double angle){}
    default void stop(){}
    
}