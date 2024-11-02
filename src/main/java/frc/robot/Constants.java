package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * All constants for the robot are stored here. In standard units below. In all
 * caps with underscores aka LEFT_MOTOR_FRONT_ID and CAN IDs and Port IDs should
 * end with "_ID"
 * 
 * Velocity: m/s
 * <p>
 * Acceleration: m/s^2
 * <p>
 * Distance: m
 * <p>
 *
 * Angle: rad or {@link edu.wpi.first.math.geometry.Rotation2d}
 * <p>
 * Angular Velocity: rad/s
 * <p>
 * Angular Acceleration: rad/s^2
 * <p>
 * 
 * Gear Ratios should be < 1 aka 1.0/8.196
 * <p>
 * 
 * Use {@link edu.wpi.first.math.util.Units} to convert to standard units
 */

public final class Constants {

  // Constants for the DriveStation
  public static class DriveStationConstants {
    public static final int DRIVE_CONTROLLER_PORT_ID = 0;
    public static final int OPERATOR_CONTROLLER_PORT_ID = 1;
    public static final int TEST_CONTROLLER_PORT_ID = 2;

    // This Seting is used to prevent the robot from being enabled when testing and
    // the code is not been validated on the hardware
    public static final boolean ALLOW_ROBOT_ENABLE = true;
  }

  // Constants for the DriveTrain subsystem
  public static class DriveTrainConstants {
    public static final int LEFT_MOTOR_FRONT_ID = 2;
    public static final int LEFT_MOTOR_BACK_ID = 3;
    public static final int RIGHT_MOTOR_FRONT_ID = 4;
    public static final int RIGHT_MOTOR_BACK_ID = 5;

    public static final double DRIVE_RADIO = (10.0/62.0) * (22.0/33.0);

    public static final double DRIVE_WHEEL_RADIUS_STOCK = Units.inchesToMeters(4.0) / 2.0;
    public static final double TREAD_WARE_FACTOR = Units.inchesToMeters(0.0);
    public static final double DRIVE_WHEEL_RADIUS = DRIVE_WHEEL_RADIUS_STOCK - TREAD_WARE_FACTOR;

    public static final double MAX_VELOCITY = Units.feetToMeters(12.89);
    public static final double TRACK_WIDTH = Units.inchesToMeters(32);
  }
}
