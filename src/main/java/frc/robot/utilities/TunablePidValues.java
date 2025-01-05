package frc.robot.utilities;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveStationConstants;

// inspired by: https://github.com/Mechanical-Advantage/RobotCode2022/blob/main/src/main/java/frc/robot/util/TunableNumber.java

public class TunablePidValues {
    private String name;
    private PIDValues values;

    public TunablePidValues (
        String name, 
        PIDValues pi
    ) {
        this.name = name;

        if (DriveStationConstants.TUNING_MODE) {
            SmartDashboard.putNumber(name + "/kP", pi.kP);
            SmartDashboard.putNumber(name + "/kI", pi.kI);
            SmartDashboard.putNumber(name + "/kD", pi.kD);
            SmartDashboard.putNumber(name + "/kFF", pi.kFF);
        }
    }

    public PIDValues get() {
        return DriveStationConstants.TUNING_MODE
            ? new PIDValues(
                (int) SmartDashboard.getNumber(name + "/kP", values.kP),
                (int) SmartDashboard.getNumber(name + "/kI", values.kI),
                (int) SmartDashboard.getNumber(name + "/kD", values.kD),
                (int) SmartDashboard.getNumber(name + "/kFF", values.kFF),
                values
            )
            : values;
    }

    // immutable representation of PID values
    public class PIDValues {
        public final int kP;
        public final int kI;
        public final int kD;
        public final int kFF;
        public final boolean updated; // true if these are different values from the old ones

        public PIDValues(int kP, int kI, int kD, int kFF) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.kFF = kFF;  
            this.updated = false; 
        }

        public PIDValues(int kP, int kI, int kD, int kFF, PIDValues oldValues) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.kFF = kFF;   
            this.updated = !oldValues.equals(this);
        }

        public boolean equals(PIDValues obj) {
            return obj.kP == kP && obj.kI == kI && obj.kD == kD && obj.kFF == kFF;
        }
    }
}
