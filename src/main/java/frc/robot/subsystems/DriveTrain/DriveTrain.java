// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DriveTrain;

import frc.robot.Constants.DriveTrainConstants;
import frc.robot.utilities.Joystick;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {

  private final DriveTrainIO io;

  private final DriveTrainIOInputsAutoLogged inputs = new DriveTrainIOInputsAutoLogged();

  private DifferentialDriveOdometry odometry;

  private DifferentialDriveKinematics kinematics;

  private Pose2d pose;

  public DriveTrain(DriveTrainIO io) {
    this.io = io;
    io.updateInputs(inputs);

    odometry = new DifferentialDriveOdometry(inputs.heading, inputs.leftFrontPosition, inputs.rightFrontPosition,
        new Pose2d(0.0, 0.0, new Rotation2d()));

    kinematics = new DifferentialDriveKinematics(DriveTrainConstants.TRACK_WIDTH);

    try{
      // Configure AutoBuilder last
      AutoBuilder.configureLTV(
              this::getPose, // Robot pose supplier
              this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
              this::getChassisSpeed, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
              this::setChassisSpeed, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
              DriveTrainConstants.DT,
              new ReplanningConfig(), // The robot configuration
              () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                  return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
              },
              this // Reference to this subsystem to set requirements
      );
    } catch (Exception e) {
      // Handle exception as needed
      DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    Logger.processInputs("DriveTrain", inputs);

    Logger.recordOutput("DriveTrain/CurentCommand",
        this.getCurrentCommand() != null ? this.getCurrentCommand().getName() : "none");

    // Update Odometry
    pose = odometry.update(inputs.heading, inputs.leftFrontPosition, inputs.leftFrontPosition);

    Logger.recordOutput("DriveTrain/Pos2d", pose);
    Logger.recordOutput("DriveTrain/WheelSpeed", this.getWheelSpeed());
    Logger.recordOutput("DriveTrain/ChassisSpeed", this.getChassisSpeed());
  }

  @Override
  public void simulationPeriodic() {
  }

  /**
   * This function is used to get ChassisSpeeds of the robot
   * 
   * @return
   */
  public ChassisSpeeds getChassisSpeed() {
    return kinematics.toChassisSpeeds(getWheelSpeed());
  }

  /**
   * This function is used to get the wheel speed of the robot
   * 
   * @return the wheel speed of the robot
   */
  public DifferentialDriveWheelSpeeds getWheelSpeed() {
    return new DifferentialDriveWheelSpeeds(inputs.leftVelocity, inputs.rightVelocity);
  }

  /**
   * This function is used to set the speed of the robot. NOTE since this is a
   * non-holonomic drive train the vyMetersPerSecond should be 0
   * 
   * @param chassisSpeed the speed of the robot
   */
  public void setChassisSpeed(ChassisSpeeds chassisSpeed) {
    DifferentialDriveWheelSpeeds wheelSpeed = kinematics.toWheelSpeeds(chassisSpeed);

    io.drive(wheelSpeed.leftMetersPerSecond / DriveTrainConstants.MAX_VELOCITY,
        wheelSpeed.rightMetersPerSecond / DriveTrainConstants.MAX_VELOCITY);
  }

  /**
   * This function is used to get the pose of the robot. This is useful for
   * PathPlanner
   * 
   * @return
   */
  public Pose2d getPose() {
    return pose;
  }

  /**
   * This function is used to reset the pose of the robot to x = 0, y = 0, and
   * theta = 0
   */
  public void resetPose(Pose2d pose) {
    System.out.println("reset Pose");
    odometry.resetPosition(inputs.heading, inputs.leftFrontPosition, inputs.rightFrontPosition,
        new Pose2d(0.0, 0.0, new Rotation2d()));
  }

  /**
   * This function is used to set the pose of the robot. This is useful for
   * setting the starting position of the robot. This is most useful for
   * PathPlanner
   * 
   * @param pose the pose of the robot
   */
  public void setPose(Pose2d pose) {
    odometry.resetPosition(inputs.heading, inputs.leftFrontPosition, inputs.rightFrontPosition, pose);
  }

  /**
   * This function is used to reset the gyro to the current heading of the robot.
   * NOTE: It does not actually reset the hardware gyro only the odometry
   */
  public void resetGyro() {
    odometry.resetPosition(inputs.heading, inputs.leftFrontPosition, inputs.rightFrontPosition,
        new Pose2d(pose.getX(), pose.getY(), new Rotation2d()));
  }

  /**
   * This Command is used to drive the robot using tank drive. WARNING: This does
   * not preform any input sanitization like
   * {@link frc.robot.subsystems.DriveTrain.DriveTrain#arcadeDriveCommand}
   * 
   * @param left  a DoubleSupplier that returns the left speed of the robot from
   *              -1 to 1
   * @param right a DoubleSupplier that returns the right speed of the robot from
   *              -1 to 1
   */
  public Command tankDriveCommand(DoubleSupplier left, DoubleSupplier right) {
    return runEnd(() -> this.io.drive(left.getAsDouble(), right.getAsDouble()), () -> this.io.drive(0, 0));
  }

  /**
   * This Command is used to drive the robot using the arcade drive.
   * 
   * @param fwd a DoubleSupplier that returns the forward speed of the robot from
   *            -1 to 1
   * @param rot a DoubleSupplier that returns the rotation speed of the robot from
   *            -1 to 1
   */
  public Command arcadeDriveCommand(DoubleSupplier fwd, DoubleSupplier rot) {
    Command cmd = runEnd(
        () -> {
          WheelSpeeds wheelSpeeds = DifferentialDrive.arcadeDriveIK(
              Joystick.JoystickInput(fwd.getAsDouble(), 2, 0.001, 1),
              -Joystick.JoystickInput(rot.getAsDouble(), 3, 0.02, 0.6),
              false);
          this.io.drive(wheelSpeeds.left, wheelSpeeds.right);
        },
        () -> this.io.drive(0, 0));
    cmd.setName("Default Arcade Drive");
    return cmd;
  }
}