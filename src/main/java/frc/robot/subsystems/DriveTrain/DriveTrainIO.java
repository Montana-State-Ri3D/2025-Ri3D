package frc.robot.subsystems.DriveTrain;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface DriveTrainIO {

    @AutoLog
    public class DriveTrainIOInputs {
        public double leftPower;
        public double rightPower;
        public boolean brake;
        public double leftPosition;
        public double rightPosition;
        public double leftVelocity;
        public double rightVelocity;
        public double leftCurrent;
        public double rightCurrent;
        public Rotation2d heading;
        
    }

    default void updateInputs(DriveTrainIOInputs inputs){}
    default void drive(double leftPower, double rightPower) {}
    default void toggleMode(){}
}
