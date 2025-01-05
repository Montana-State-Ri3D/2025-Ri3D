package frc.robot.subsystems.Arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
       
    @AutoLog
    public class ArmIOInputs{
        public double elevatorMasterPower;
        public double elevatorSlavePower;

        public double elevatorMasterCurrent;
        public double elevatorSlaveCurrent;

        public double elevatorMasterEncoder;
        public double elevatorSlaveEncoder;

        public double elevatorMasterPosition;
        public double elevatorSlavePosition;

        public double elevatorMasterVelocity;
        public double elevatorSlaveVelocity;

        
        public boolean isBrake;
    }
    default void updateInputs(ArmIOInputs inputs){}
    default void setElevatorPower(double elevatorPower){}
    default void setBrake(boolean isBrake){}
}