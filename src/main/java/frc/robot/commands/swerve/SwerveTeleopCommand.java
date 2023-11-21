package frc.robot.commands.swerve;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utils.Constants;

/***
 * @author Noah Simon
 * @author Raadwan Masum
 * @author Rohin Sood
 *         Default command to control the SwervedriveSubsystem with joysticks
 */

 
public class SwerveTeleopCommand extends CommandBase {

    private final Swerve swerve;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    public SwerveTeleopCommand(
            Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction) {
        this.swerve = Robot.swerve;
        this.xSpdFunction = () -> 0.;
        this.ySpdFunction = () -> 0.;
        this.turningSpdFunction = () -> 0.;
        this.xLimiter = new SlewRateLimiter(Constants.Swerve.maxDriveAccelerationMPSS);
        this.yLimiter = new SlewRateLimiter(Constants.Swerve.maxDriveAccelerationMPSS);
        this.turningLimiter = new SlewRateLimiter(Constants.Swerve.maxRotationAccelerationRadPSS);
        addRequirements(swerve);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

        // 1. Get real-time joystick inputs
        double xSpeed = xSpdFunction.get();
        double ySpeed = -ySpdFunction.get();
        double yOpSpeed = -ySpdFunction.get();
        double turningSpeed = -turningSpdFunction.get();

        // 2. Apply deadband
        xSpeed = Math.abs(xSpeed) > Constants.Swerve.controllerDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > Constants.Swerve.controllerDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > Constants.Swerve.controllerDeadband ? turningSpeed : 0.0;

        // 3. Make the driving smoother
        xSpeed = xLimiter.calculate(xSpeed) * Constants.Swerve.maxDriveSpeedMPS;
        ySpeed = yLimiter.calculate(ySpeed) * Constants.Swerve.maxDriveSpeedMPS;
        turningSpeed = turningLimiter.calculate(turningSpeed)
                * Constants.Swerve.maxRotationSpeedRadPS;

        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;

        // Relative to field
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, turningSpeed, swerve.getRotation2d());

        // Relative to robot
        // chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);

        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = Constants.Swerve.driveKinematics.toSwerveModuleStates(chassisSpeeds);

        // 6. Output each module states to wheels
        swerve.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}