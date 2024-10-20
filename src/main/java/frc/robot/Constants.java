// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class DriveStationConstants {
    public static final int DRIVE_CONTROLLER_PORT = 0;
  }
  public static class DriveTrainConstants {
    public static final int LEFT_MOTOR_FRONT = 0;
    public static final int LEFT_MOTOR_BACK = 1;
    public static final int RIGHT_MOTOR_FRONT = 2;
    public static final int RIGHT_MOTOR_BACK = 3;

    public static final double DRIVE_RADIO = 1.0/32.0;

    public static final double DRIVE_WHEEL_RADIUS_STOCK = Units.inchesToMeters(2.0);
    public static final double TREAD_WARE_FACTOR = Units.inchesToMeters(0.0);
    public static final double DRIVE_WHEEL_RADIUS = DRIVE_WHEEL_RADIUS_STOCK - TREAD_WARE_FACTOR;

    public static final double MAX_VELOCITY = 3.0;
    public static final double TRACK_WIDTH = Units.inchesToMeters(32);


  }
}
