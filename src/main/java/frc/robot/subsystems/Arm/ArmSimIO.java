package frc.robot.subsystems.Arm;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;

// import com.revrobotics.sim.SparkMaxSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm.Arm.ArmPosition;



public class ArmSimIO implements ArmIO {
    private final PWMSparkMax elevatorLeader;
    private final PWMSparkMax elevatorFollower;

    // private final SparkMAXSim elevatorLeaderSim;

    private final Encoder elevatorLeaderEncoder;
    private final Encoder elevatorFollowerEncoder;
    private final EncoderSim elevatorLeaderEncoderSim;
    private final EncoderSim elevatorFollowerEncoderSim;

    private final ElevatorSim elevatorSim;

    private ProfiledPIDController elevatorPIDController = new ProfiledPIDController(
        ArmConstants.ELEVATOR_kP,
        ArmConstants.ELEVATOR_kI,
        ArmConstants.ELEVATOR_kD,
        new TrapezoidProfile.Constraints(2.45, 2.45)
    );

    private double elevatorTargetPosition;

    public ArmSimIO() {
        elevatorLeader = new PWMSparkMax(5);
        elevatorLeaderEncoder = new Encoder(6, 7);
        elevatorLeaderEncoderSim = new EncoderSim(elevatorLeaderEncoder);

        elevatorFollower = new PWMSparkMax(8);
        elevatorFollowerEncoder = new Encoder(9, 10);
        elevatorFollowerEncoderSim = new EncoderSim(elevatorFollowerEncoder);

        elevatorSim = new ElevatorSim(
            DCMotor.getNEO(2),
            ArmConstants.ELEVATOR_RATIO,
            5,
            20,
            0,
            2,
            true,
            0
        );
    }

    public void setElevatorPos(double elevatorPos) {
        elevatorTargetPosition = elevatorPos;
    }

    public void updateInputs(ArmIOInputs inputs) {

        elevatorSim.setInput(
            elevatorLeader.get() * RobotController.getBatteryVoltage()
        );

        reachGoal(elevatorTargetPosition);

        elevatorSim.update(0.02);

        elevatorLeaderEncoderSim.setDistance(elevatorSim.getPositionMeters());
        elevatorFollowerEncoderSim.setDistance(elevatorSim.getPositionMeters());

        elevatorFollowerEncoderSim.setRate(elevatorSim.getVelocityMetersPerSecond());
        elevatorLeaderEncoderSim.setRate(elevatorSim.getVelocityMetersPerSecond());

        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));

        inputs.elevatorLeaderPower = this.elevatorLeader.get();
        inputs.elevatorFollowerPower = this.elevatorFollower.get();

        inputs.elevatorLeaderPosition = this.elevatorLeaderEncoder.getDistance();
        inputs.elevatorFollowerPosition = this.elevatorFollowerEncoder.getDistance();

        inputs.elevatorTargetPosition = elevatorTargetPosition;

        inputs.elevatorLeaderVelocity = this.elevatorLeaderEncoder.getRate();
        inputs.elevatorFollowerVelocity = this.elevatorFollowerEncoder.getRate();

        inputs.elevatorLeaderCurrent = this.elevatorLeader.get();
    //     inputs.elevatorFollowerCurrent = this.elevatorFollower.getOutputCurrent();


    //     inputs.limitSwitchHit = limitSwitch.get();
    }

    public void reachGoal(double goal) {
        elevatorPIDController.setGoal(goal);
    
        // With the setpoint value we run PID control like normal
        double pidOutput = elevatorPIDController.calculate(elevatorLeaderEncoder.getDistance());
        elevatorLeader.set(pidOutput);
      }
}
