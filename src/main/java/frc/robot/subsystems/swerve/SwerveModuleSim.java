package frc.robot.subsystems.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.utils.Constants;

public class SwerveModuleSim implements SwerveModuleIO {

    private FlywheelSim driveSim = new FlywheelSim(DCMotor.getNEO(1), 6.75, 0.025);
    private FlywheelSim turningSim = new FlywheelSim(DCMotor.getNEO(1), 150 / 7, 0.004);

    private PIDController drivePID= new PIDController(0, 0, 0);
    private PIDController turnPID = new PIDController(0, 0, 0);
    private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0, 0, 0);

    private SwerveModulePosition position = new SwerveModulePosition();
    private SwerveModuleState theoreticalState = new SwerveModuleState();

    private double drivePositionM = 0.0;
    private double driveAppliedVolts = 0.0;

    private double turnPositionRad = 0.0;
    private double turnAppliedVolts = 0.0;

    public SwerveModuleSim() {
        turnPID.enableContinuousInput(0, 2 * Math.PI);
    }

    /** Updates the set of loggable inputs. */
    @Override
    public void updateData(ModuleData data) {
        driveSim.update(0.02);
        turningSim.update(0.02);

        drivePositionM += driveSim.getAngularVelocityRadPerSec() * 0.02 * Constants.Swerve.wheelDiameterM / 2;
        data.drivePositionM = drivePositionM;
        data.driveVelocityMPerSec = driveSim.getAngularVelocityRadPerSec() * Constants.Swerve.wheelDiameterM / 2;
        data.driveAppliedVolts = driveAppliedVolts;
        data.driveCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());

        double angleDiff = turningSim.getAngularVelocityRadPerSec() * 0.02;
        while (turnPositionRad < 0) {
            turnPositionRad += 2.0 * Math.PI;
        }
        while (turnPositionRad > 2.0 * Math.PI) {
            turnPositionRad -= 2.0 * Math.PI;
        }
        turnPositionRad += angleDiff;

        data.turnPositionRad = turnPositionRad;
        data.turnVelocityRadPerSec = turningSim.getAngularVelocityRadPerSec();
        data.turnAppliedVolts = turnAppliedVolts;
        data.turnCurrentAmps = Math.abs(turningSim.getCurrentDrawAmps());
        data.turnPositionRad = turningSim.getAngularVelocityRadPerSec() * 0.02;

        data.theoreticalState = theoreticalState;
        data.position = position;
    }

    /** Run the drive motor at the specified voltage. */
    @Override
    public void setDesiredState(SwerveModuleState state) {
        theoreticalState = state;
        
       state = SwerveModuleState.optimize(state, state.angle);

        double driveVolt = drivePID.calculate(driveSim.getAngularVelocityRadPerSec() * Constants.Swerve.wheelDiameterM / 2,
                state.speedMetersPerSecond) + driveFeedforward.calculate(state.speedMetersPerSecond);
        double turnVolt = turnPID.calculate(turningSim.getAngularVelocityRadPerSec(),
                state.angle.getRadians());

        driveSim.setInputVoltage(driveVolt);
        turningSim.setInputVoltage(turnVolt);
    }

    /** Run the turn motor at the specified voltage. */
    @Override
    public void stop() {
        driveSim.setInputVoltage(0.0);
        turningSim.setInputVoltage(0.0);
    }

    /** Enable or disable brake mode on the drive motor. */
    @Override
    public void setDriveBrakeMode(boolean enable) {
    }

    /** Enable or disable brake mode on the turn motor. */
    @Override
    public void setTurningBrakeMode(boolean enable) {
    }
}
