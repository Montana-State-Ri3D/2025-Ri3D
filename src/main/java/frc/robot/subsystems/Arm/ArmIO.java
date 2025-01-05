package frc.robot.subsystems.Arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
       
    @AutoLog
    public class ArmIOInputs{
        public double elevatorLeaderPower;
        public double elevatorFollowerPower;

        public double elevatorLeaderCurrent;
        public double elevatorFollowerCurrent;

        public double elevatorLeaderEncoder;
        public double elevatorFollowerEncoder;

        public double elevatorLeaderPosition;
        public double elevatorFollowerPosition;

        public double elevatorLeaderVelocity;
        public double elevatorFollowerVelocity;

        public double elevatorTargetPosition;

        public boolean isBrake;
    }
    default void updateInputs(ArmIOInputs inputs){}
    default void setElevatorPower(double elevatorPower){}
    default void setBrake(boolean isBrake){}
}