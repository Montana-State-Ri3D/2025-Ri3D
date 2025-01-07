package frc.robot.subsystems.EndEffector;

import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {
    @AutoLog
    public class EndEffectorIOInputs {
        /**
         * The power of the wheels in the end effector
         */
        public double power;

        /**
         * The current of the left wheel in the end effector
         */
        public double current;
    }

    /**
     * updateInputs: Updates the inputs of the end effector subsystem
     * 
     * @param inputs
     */
    default void updateInputs(EndEffectorIOInputs inputs) {}

    default void setPower(double power) {}
}
