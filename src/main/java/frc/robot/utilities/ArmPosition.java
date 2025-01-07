package frc.robot.utilities;

public class ArmPosition {
    public final int elevatorPosition;
    public final int elbowPosition;
    public final int wristPosition;

    public ArmPosition(int elevatorPosition, int elbowPosition, int wristPosition) {
        this.elevatorPosition = elevatorPosition;
        this.elbowPosition = elbowPosition;
        this.wristPosition = wristPosition;
    }
}
