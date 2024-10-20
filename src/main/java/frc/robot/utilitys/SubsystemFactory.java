package frc.robot.utilitys;

import frc.robot.Robot;

import frc.robot.Constants.DriveTrainConstants;
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
                    new DriveTrainRealIO(DriveTrainConstants.LEFT_MOTOR_BACK, DriveTrainConstants.LEFT_MOTOR_FRONT,
                            DriveTrainConstants.RIGHT_MOTOR_BACK, DriveTrainConstants.RIGHT_MOTOR_FRONT));
        } else {
            return new DriveTrain(new DriveTrainSimIO());
        }
    }

}
