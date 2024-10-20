package frc.robot.utilitys;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.robot.Robot;

public class AdvantageKitHelper {

    @SuppressWarnings("resource")
    public static void setupLogger() {
        if (Robot.isReal()) {
            Logger.addDataReceiver(new WPILOGWriter());
            Logger.addDataReceiver(new NT4Publisher());
            new PowerDistribution(1, ModuleType.kRev);
        } else {
            Logger.addDataReceiver(new NT4Publisher());
        }

        Logger.start();
    }
}
