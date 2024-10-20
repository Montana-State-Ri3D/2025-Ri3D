package frc.robot.utilities;

public class Joystick {

    /**
     * Used to sanitize joystick input
     * 
     * @param value    value to sanitize
     * @param power    power to raise the value to
     * @param deadband deadband to apply to the value
     * @param clamp    clamp to set min and max values
     * @return sanitized value
     */
    public static double JoystickInput(double value, double power, double deadband, double clamp) {
        if (power != 0) {
            value = Math.copySign(Math.pow(value, power), value);
        }
        if (deadband != 0) {
            if (Math.abs(value) > deadband) {
                if (value > 0) {
                    value = (value - deadband) / (1.0 - deadband);
                } else {
                    value = (value + deadband) / (1.0 - deadband);
                }
            } else {
                value = 0;
            }
        }
        if (clamp != 1) {
            value = Math.max(-clamp, Math.min(value, clamp));
        }
        return value;
    }

    /**
     * Used to sanaize joystick input with a power of 2, deadband of 0.02, and clamp
     * of 1.0
     * 
     * @param value value to sanitize
     * @return sanitized value
     */
    public static double JoystickInput(double value) {
        return JoystickInput(value, 2, 0.02, 1.0);
    }

    /**
     * Used to sanaize joystick input with a power of 2, and clamp of 1.0
     * 
     * @param value    value to sanitize
     * @param deadband deadband to apply to the value
     * @return sanitized value
     */

    public static double JoystickInput(double value, double deadband) {
        return JoystickInput(value, 2, deadband, 1.0);
    }
}