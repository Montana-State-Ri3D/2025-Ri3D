package frc.robot.subsystems.DriveTrain;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface DriveTrainIO {

    @AutoLog
    public class DriveTrainIOInputs {
        /**
         * leftPower: The power of the left side of the drive train from -1 to 1
         */
        public double leftPower;
        /**
         * rightPower: The power of the right side of the drive train from -1 to 1
         */
        public double rightPower;
        /**
         * brake: Whether or not the drive train is in brake mode
         */
        public boolean brake;
        /**
         * leftFrontPosition: The position of the left front side of the drive train in meters
         */
        public double leftFrontPosition;
        /**
         * rightFrontPosition: The position of the right front side of the drive train in meters
         */
        public double rightFrontPosition;
        /**
         * leftBackPosition: The position of the left back side of the drive train in meters
         */
        public double leftBackPosition;
        /**
         * rightBackPosition: The position of the right back side of the drive train in meters
         */
        public double rightBackPosition;
        /**
         * leftVelocity: The velocity of the left side of the drive train in meters per
         * second
         */
        public double leftVelocity;
        /**
         * rightVelocity: The velocity of the right side of the drive train in meters
         * per second
         */
        public double rightVelocity;
        /**
         * leftCurrent: The current draw of the left side of the drive train in amps
         */
        public double leftCurrent;
        /**
         * rightCurrent: The current draw of the right side of the drive train in amps
         */
        public double rightCurrent;
        /**
         * heading: The heading of the robot as
         * {@link edu.wpi.first.math.geometry.Rotation2d}
         */
        public Rotation2d heading;

    }

    /**
     * updateInputs: Updates the inputs of the drivetrain subsystem
     * 
     * @param inputs
     */
    default void updateInputs(DriveTrainIOInputs inputs) {
    }

    /**
     * drive: Drives the robot with the given power values
     * 
     * @param leftPower:  The power of the left side of the drive train from -1 to 1
     * @param rightPower: The power of the right side of the drive train from -1 to
     *                    1
     */
    default void drive(double leftPower, double rightPower) {
    }

    /**
     * brake: Toggles the DriveTrain between brake and coast mode
     */
    default void toggleMode() {
    }
}
