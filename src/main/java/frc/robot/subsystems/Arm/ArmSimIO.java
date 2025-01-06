package frc.robot.subsystems.Arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants.ArmConstants;
import frc.robot.utilities.TunablePidValues;
import frc.robot.utilities.TunablePidValues.PIDValues;

public class ArmSimIO implements ArmIO {

    private final PIDController elevatorPIDController;
    private final TunablePidValues elevatorTunablePid;

    // simulation fields
    private final ElevatorSim elevatorSim;

    public ArmSimIO() {
        elevatorTunablePid = new TunablePidValues(
            "Elevator PID",
            new PIDValues(
                ArmConstants.ELEVATOR_kP, 
                ArmConstants.ELEVATOR_kI, 
                ArmConstants.ELEVATOR_kD, 
                0
            )
        );

        elevatorSim = new ElevatorSim(
            DCMotor.getNEO(2),
            ArmConstants.ELEVATOR_RATIO,
            20,
            0.05,
            0,
            2,
            false,
            1
        );

        elevatorPIDController = new PIDController(
            ArmConstants.ELEVATOR_kP,
            ArmConstants.ELEVATOR_kI,
            ArmConstants.ELEVATOR_kD
        );
    }

    public void setElevatorPos(double elevatorPos) {
        elevatorPIDController.setSetpoint(elevatorPos);
    }

    public void updateInputs(ArmIOInputs inputs) {
        // update the simulation here since this is called periodicly
        updateElevatorSim();
        
        // perform updates to inputs as usual
        inputs.elevatorLeaderPower = elevatorSim.getCurrentDrawAmps();
        inputs.elevatorFollowerPower = elevatorSim.getCurrentDrawAmps();

        inputs.elevatorLeaderPosition = elevatorSim.getPositionMeters();
        inputs.elevatorFollowerPosition = elevatorSim.getPositionMeters();

        inputs.elevatorTargetPosition = elevatorPIDController.getSetpoint();

        inputs.elevatorLeaderVelocity = elevatorSim.getVelocityMetersPerSecond();
        inputs.elevatorFollowerVelocity = elevatorSim.getVelocityMetersPerSecond();
    }

    public void updatePIDValues() {
        PIDValues elevatorPidValues = elevatorTunablePid.get();

        if (elevatorPidValues.updated) {
            elevatorPIDController.setP(elevatorPidValues.kP);
            elevatorPIDController.setI(elevatorPidValues.kI);
            elevatorPIDController.setD(elevatorPidValues.kD);
        }
    }

    // update the simulated elevator
    private void updateElevatorSim() {
        double pidOutput = elevatorPIDController.calculate(elevatorSim.getPositionMeters());

        elevatorSim.setInputVoltage(pidOutput * 12); // hardcode the battery voltage
        elevatorSim.update(0.02);
    }
}
