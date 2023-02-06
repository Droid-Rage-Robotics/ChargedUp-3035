package frc.robot.commands.Drive;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.DroidRageConstants;
import frc.robot.subsystems.SwerveModule;

public class SwerveDriveTeleop extends CommandBase {
    public static class Constants {
        public static final double MAX_ACCELERATION_UNITS_PER_SECOND = 3;
        public static final double MAX_ANGULAR_ACCELERATION_UINTS_PER_SECOND = 3;
        
        public static final double MAX_SPEED_METERS_PER_SECOND = SwerveModule.Constants.PHYSICAL_MAX_SPEED_METERS_PER_SECOND / 4;
        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND= SwerveModule.Constants.PHYSICAL_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND / 4;
    }

    private final Drive drive;
    private final Supplier<Double> x, y, turn;
    private final Supplier<Boolean> isFieldOriented;
    private final SlewRateLimiter xLimiter, yLimiter, turnLimiter;
    public SwerveDriveTeleop(Drive drive,
            Supplier<Double> x, Supplier<Double> y, Supplier<Double> turn,
            Supplier<Boolean> isFieldOriented) {
        this.drive = drive;
        this.x = x;
        this.y = y;
        this.turn = turn;
        this.isFieldOriented = isFieldOriented;
        this.xLimiter = new SlewRateLimiter(Constants.MAX_ACCELERATION_UNITS_PER_SECOND);
        this.yLimiter = new SlewRateLimiter(Constants.MAX_ACCELERATION_UNITS_PER_SECOND);
        this.turnLimiter = new SlewRateLimiter(Constants.MAX_ANGULAR_ACCELERATION_UINTS_PER_SECOND);

        addRequirements(drive);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double xSpeed = x.get();
        double ySpeed = y.get();
        double turnSpeed = turn.get();

        xSpeed = Math.abs(xSpeed) > DroidRageConstants.Gamepad.STICK_DEADZONE ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > DroidRageConstants.Gamepad.STICK_DEADZONE ? ySpeed : 0.0;
        turnSpeed = Math.abs(turnSpeed) > DroidRageConstants.Gamepad.STICK_DEADZONE ? turnSpeed : 0.0;

        xSpeed = xLimiter.calculate(xSpeed) * Constants.MAX_SPEED_METERS_PER_SECOND;
        ySpeed = yLimiter.calculate(ySpeed) * Constants.MAX_SPEED_METERS_PER_SECOND;
        turnSpeed = turnLimiter.calculate(turnSpeed) * Constants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;

        ChassisSpeeds chassisSpeeds;
        if (isFieldOriented.get()) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, turnSpeed, drive.getRotation2d()
            );
        } else {
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turnSpeed);
        }

        SwerveModuleState[] states = Drive.Constants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

        drive.setModuleStates(states);
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
