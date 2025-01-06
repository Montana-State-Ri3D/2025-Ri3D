package frc.robot.utilities;

import frc.robot.Robot;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.ArmIO;
import frc.robot.subsystems.Arm.ArmRealIO;
import frc.robot.subsystems.Arm.ArmSimIO;
import frc.robot.subsystems.DriveTrain.DriveTrain;
import frc.robot.subsystems.DriveTrain.DriveTrainRealIO;
import frc.robot.subsystems.DriveTrain.DriveTrainSimIO;

public class SubsystemFactory {
    boolean isReal = false;

    public SubsystemFactory() {
        isReal = Robot.isReal();
    }

    public DriveTrain createDriveTrain() {
        if (isReal) {
            return new DriveTrain(
                    new DriveTrainRealIO(
                            DriveTrainConstants.LEFT_MOTOR_BACK_ID,
                            DriveTrainConstants.LEFT_MOTOR_FRONT_ID,
                            DriveTrainConstants.RIGHT_MOTOR_BACK_ID,
                            DriveTrainConstants.RIGHT_MOTOR_FRONT_ID,
                            DriveTrainConstants.PIGEON_ID
                            ));
        } else {
            return new DriveTrain(new DriveTrainSimIO());
        }
    }

    public Arm createArm() {
        if (isReal) {
            return new Arm(
                new ArmRealIO(
                    ArmConstants.ELEVATOR_LEADER_ID,
                    ArmConstants.ELEVATOR_FOLLOWER_ID,
                    ArmConstants.ELBOW_LEADER_ID,
                    ArmConstants.ELBOW_FOLLOWER_ID,
                    ArmConstants.WRIST_ID,
                    ArmConstants.LIMIT_SWITCH_ID
                )
            );
        } else {
            return new Arm(new ArmSimIO());
        }
    }

}
