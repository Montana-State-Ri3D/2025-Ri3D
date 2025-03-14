package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;


public interface IntakeIO {
    
    @AutoLog
    public class IntakeIOInputs{
        public double rightPower;

        public double leftPower;

        public double pivotPower;

        public double rightCurrent;

        public double leftCurrent;

        public double pivotCurrent;

        public boolean isBrake;

        public double leftVelo;

        public double rightVelo;

        public double pivotVelo;

        public double pivotAngle;

        public double targetAngle;

        public Intake.IntakePosition position;

        public boolean hasCoral;

        public boolean hasAlgae;
    }
    
    default void updateInputs(IntakeIOInputs inputs){}
    default void setPower(double leftPower, double rightPower){}
    default void setPivotPower(double power){}
    default void setBrake(boolean isBrake){}
    default void setAngle(double angle){}
    default void stop(){}
    
}