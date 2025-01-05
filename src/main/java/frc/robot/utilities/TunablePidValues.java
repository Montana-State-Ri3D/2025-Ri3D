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
        this.values = pi;

        if (DriveStationConstants.TUNING_MODE) {
            SmartDashboard.putNumber(name + "/kP", pi.kP);
            SmartDashboard.putNumber(name + "/kI", pi.kI);
            SmartDashboard.putNumber(name + "/kD", pi.kD);
            SmartDashboard.putNumber(name + "/kFF", pi.kFF);
        }
    }

    public PIDValues get() {
        PIDValues newValues = DriveStationConstants.TUNING_MODE
            ? new PIDValues(
                (double) SmartDashboard.getNumber(name + "/kP", values.kP),
                (double) SmartDashboard.getNumber(name + "/kI", values.kI),
                (double) SmartDashboard.getNumber(name + "/kD", values.kD),
                (double) SmartDashboard.getNumber(name + "/kFF", values.kFF),
                values
            )
            : values;

        values = newValues;
        return newValues;
    }

    // immutable representation of PID values
    public static class PIDValues {
        public final double kP;
        public final double kI;
        public final double kD;
        public final double kFF;
        public final boolean updated; // true if these are different values from the old ones

        public PIDValues(double d, double e, double f, double g) {
            this.kP = d;
            this.kI = e;
            this.kD = f;
            this.kFF = g;  
            this.updated = false; 
        }

        public PIDValues(double kP, double kI, double kD, double kFF, PIDValues oldValues) {
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
