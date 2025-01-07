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

    // enable PID tuning from smart dashboard
    public static final boolean TUNING_MODE = true;
  }

  // Constants for the DriveTrain subsystem
  public static class DriveTrainConstants {
    public static final int LEFT_MOTOR_FRONT_ID = 2;
    public static final int LEFT_MOTOR_BACK_ID = 3;
    public static final int RIGHT_MOTOR_FRONT_ID = 4;
    public static final int RIGHT_MOTOR_BACK_ID = 5;

    public static final int PIGEON_ID = 6;

    public static final double DRIVE_RATIO = (10.0/62.0) * (22.0/33.0);

    public static final double DRIVE_WHEEL_RADIUS_STOCK = Units.inchesToMeters(4.0) / 2.0;
    public static final double TREAD_WARE_FACTOR = Units.inchesToMeters(0.0);
    public static final double DRIVE_WHEEL_RADIUS = DRIVE_WHEEL_RADIUS_STOCK - TREAD_WARE_FACTOR;

    public static final double MAX_VELOCITY = Units.feetToMeters(11.23);
    public static final double TRACK_WIDTH = Units.inchesToMeters(24);
  }

  // Constants for the Arm subsystem
  public static class ArmConstants {
    // Motor ID's
    public static final int ELEVATOR_LEADER_ID = 7;
    public static final int ELEVATOR_FOLLOWER_ID = 8;

    public static final int ELBOW_LEADER_ID = 9;
    public static final int ELBOW_FOLLOWER_ID = 10;

    public static final int WRIST_ID = 11;

    // Sensors
    public static final int LIMIT_SWITCH_ID = 0;

    // Offsets
    public static final double ELBOW_OFFSET = 0.15;
    public static final double WRIST_OFFSET = 0.89;

    // PID values
    public static final double ELEVATOR_kP = 0.05;
    public static final double ELEVATOR_kI = 0.0;
    public static final double ELEVATOR_kD = 1.0;

    public static final double ELBOW_kP = 0.7;
    public static final double ELBOW_kI = 0.0;
    public static final double ELBOW_kD = 0.0;

    public static final double WRIST_kP = 0.5;
    public static final double WRIST_kI = 0.0005;
    public static final double WRIST_kD = 1.0;

    public static final float ELEVATOR_HEIGHT = 47;

    // Gearbox Ratios
    public static final double ELEVATOR_RATIO = (1.0/20.0);
    public static final double ELBOW_RATIO = 0.0;
    public static final double WRIST_RATIO = 0.0;

    // Diameters
    public static final double ELEVATOR_SPROCKET_RADIUS = 1.79 / 2.0;
  }
}
