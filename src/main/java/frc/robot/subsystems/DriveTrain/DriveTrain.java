// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DriveTrain;

import frc.robot.Constants.DriveTrainConstants;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
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

    odometry = new DifferentialDriveOdometry(inputs.heading, inputs.leftPosition, inputs.rightPosition,
        new Pose2d(0.0, 0.0, new Rotation2d()));

    kinematics = new DifferentialDriveKinematics(DriveTrainConstants.TRACK_WIDTH);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    Logger.processInputs("DriveTrain", inputs);

    if (this.getCurrentCommand() != null) {

      Logger.recordOutput("DriveTrain/CurentCommand", this.getCurrentCommand().getName());
    } else {
      Logger.recordOutput("DriveTrain/CurentCommand", "none");
    }

    // Update Odometry
    pose = odometry.update(inputs.heading, inputs.leftPosition, inputs.leftPosition);

    Logger.recordOutput("DriveTrain/Pos2d", pose);

    Logger.recordOutput("DriveTrain/WheelSpeed", this.getWheelSpeed());

    Logger.recordOutput("DriveTrain/ChassisSpeed", this.getChassisSpeed());
  }

  @Override
  public void simulationPeriodic() {
  }

  public ChassisSpeeds getChassisSpeed() {

    return kinematics.toChassisSpeeds(getWheelSpeed());
  }

  public DifferentialDriveWheelSpeeds getWheelSpeed() {
    return new DifferentialDriveWheelSpeeds(inputs.leftVelocity, inputs.rightVelocity);
  }

  public void setChassisSpeed(ChassisSpeeds chassisSpeed) {
    DifferentialDriveWheelSpeeds wheelSpeed = kinematics.toWheelSpeeds(chassisSpeed);

    io.drive(wheelSpeed.leftMetersPerSecond / DriveTrainConstants.MAX_VELOCITY,
        wheelSpeed.rightMetersPerSecond / DriveTrainConstants.MAX_VELOCITY);
  }

  public Pose2d getPose() {
    return pose;
  }

  public void resetPose() {
    odometry.resetPosition(inputs.heading, inputs.leftPosition, inputs.rightPosition,
        new Pose2d(0.0, 0.0, new Rotation2d()));
  }

  public void setPose(Pose2d pose) {
    odometry.resetPosition(inputs.heading, inputs.leftPosition, inputs.rightPosition, pose);
  }

  public void resetGyro() {
    odometry.resetPosition(inputs.heading, inputs.leftPosition, inputs.rightPosition,
        new Pose2d(pose.getX(), pose.getY(), new Rotation2d()));
  }

  public Command driveCommand(DoubleSupplier left, DoubleSupplier right) {
    return runEnd(() -> this.io.drive(left.getAsDouble(), right.getAsDouble()), () -> this.io.drive(0, 0));
  }
}