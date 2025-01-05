package frc.robot.subsystems.Arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
       
    @AutoLog
    public class ArmIOInputs{
        public double elevatorLeaderPower;
        public double elevatorFollowerPower;
        public double elbowLeaderPower;
        public double elbowFollowerPower;
        public double wristPower;

        public double elevatorLeaderCurrent;
        public double elevatorFollowerCurrent;
        public double elbowLeaderCurrent;
        public double elbowFollowerCurrent;
        public double wristCurrent;

        public double elevatorLeaderPosition;
        public double elevatorFollowerPosition;
        public double elbowPosition;
        public double wristPosition;

        public double elevatorLeaderVelocity;
        public double elevatorFollowerVelocity;
        public double elbowVelocity;
        public double wristVelocity;

        public double elevatorTargetPosition;
        public double elbowTargetPosition;
        public double wristTargetPosition;

        public boolean isBrake;
    }
    default void updateInputs(ArmIOInputs inputs){}
    default void setElevatorPower(double elevatorPower){}
    default void setBrake(boolean isBrake){}
}