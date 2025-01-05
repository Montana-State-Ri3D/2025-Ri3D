package frc.robot.subsystems.EndEffector;

import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {
    @AutoLog
    public class EndEffectorIOInputs {
        /**
         * The power of the wheels in the end effector
         */
        public double power;
    }

    /**
     * updateInputs: Updates the inputs of the end effector subsystem
     * 
     * @param inputs
     */
    default void updateInputs(EndEffectorIOInputs inputs) {}
}
