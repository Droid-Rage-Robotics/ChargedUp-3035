package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Gamepad;
import frc.robot.subsystems.SwerveModule;

public class SwerveDriveCommand extends CommandBase {
    public static class Constants {
        public static final double MAX_ACCELERATION_UNITS_PER_SECOND = 3;
        public static final double MAX_ANGULAR_ACCELERATION_UINTS_PER_SECOND = 3;
        
        public static final double MAX_SPEED_METERS_PER_SECOND = SwerveModule.Constants.PHYSICAL_MAX_SPEED_METERS_PER_SECOND / 4;
        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND= SwerveModule.Constants.PHYSICAL_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND / 4;
    }

    private final SwerveDrive drive;
    private final Supplier<Double> x, y, turn;
    private final Supplier<Boolean> isFieldOriented;
    private final SlewRateLimiter xLimiter, yLimiter, turnLimiter;
    public SwerveDriveCommand(SwerveDrive drive,
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

        xSpeed = Math.abs(xSpeed) > Gamepad.Constants.DEADZONE ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > Gamepad.Constants.DEADZONE ? ySpeed : 0.0;
        turnSpeed = Math.abs(turnSpeed) > Gamepad.Constants.DEADZONE ? turnSpeed : 0.0;

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

        SwerveModuleState[] states = SwerveDrive.Constants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

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
