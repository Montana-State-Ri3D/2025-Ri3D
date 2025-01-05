package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.MoveArm;
import frc.robot.commands.MoveArm.ArmPreset;
import frc.robot.subsystems.Arm.Arm;

public class GoToConfiguration extends SequentialCommandGroup {
    public enum RobotConfiguration {
        INTAKE_CORAL,
        INTAKE_ALGAE,
        PROCESSOR
    }

    public GoToConfiguration(RobotConfiguration config, Arm arm) {
        switch (config) {
            case INTAKE_CORAL:
                addCommands(
                    new MoveArm(ArmPreset.CORAL, arm)
                );

                break;
            case INTAKE_ALGAE:
                // Add commands to move the arm to the intake algae configuration
                break;
            case PROCESSOR:
                // Add commands to move the arm to the processor configuration
                break;
        }
    }
}
