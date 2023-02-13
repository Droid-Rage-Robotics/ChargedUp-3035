package frc.robot.commands.Drive;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.SwerveModule;
import frc.robot.DroidRageConstants;

public class SwerveDriveTeleop extends CommandBase {
    private final Drive drive;
    private final Supplier<Double> x, y, turn;
    private final SlewRateLimiter xLimiter, yLimiter, turnLimiter;
    private final PIDController antiTipX, antiTipY, autoBalanceX, autoBalanceY;
    private volatile double xSpeed, ySpeed, turnSpeed;
    public SwerveDriveTeleop(Drive drive,
            Supplier<Double> x, Supplier<Double> y, Supplier<Double> turn) {
        this.drive = drive;
        this.x = x;
        this.y = y;
        this.turn = turn;

        this.xLimiter = new SlewRateLimiter(Drive.TeleOpConstants.MAX_ACCELERATION_UNITS_PER_SECOND);
        this.yLimiter = new SlewRateLimiter(Drive.TeleOpConstants.MAX_ACCELERATION_UNITS_PER_SECOND);
        this.turnLimiter = new SlewRateLimiter(Drive.TeleOpConstants.MAX_ANGULAR_ACCELERATION_UINTS_PER_SECOND);

        antiTipX = new PIDController(Drive.TeleOpConstants.getAntiTippingXP(), 0, Drive.TeleOpConstants.getAntiTippingXD());
        antiTipY = new PIDController(Drive.TeleOpConstants.getAntiTippingYP(), 0, Drive.TeleOpConstants.getAntiTippingYD());
        autoBalanceX = new PIDController(Drive.TeleOpConstants.getAutoBalanceXP(), 0, Drive.TeleOpConstants.getAutoBalanceXD());
        autoBalanceY = new PIDController(Drive.TeleOpConstants.getAutoBalanceYP(), 0, Drive.TeleOpConstants.getAutoBalanceYD());

        addRequirements(drive);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        // Get PID values from dashboard
        antiTipX.setPID(Drive.TeleOpConstants.getAntiTippingXP(), 0, Drive.TeleOpConstants.getAntiTippingXD());
        antiTipY.setPID(Drive.TeleOpConstants.getAntiTippingYP(), 0, Drive.TeleOpConstants.getAntiTippingYD());
        autoBalanceX.setPID(Drive.TeleOpConstants.getAutoBalanceXP(), 0, Drive.TeleOpConstants.getAutoBalanceXD());
        autoBalanceY.setPID(Drive.TeleOpConstants.getAutoBalanceYP(), 0, Drive.TeleOpConstants.getAutoBalanceYD());

        xSpeed = x.get();
        ySpeed = -y.get();
        turnSpeed = turn.get();

        // Square inputs
        if (drive.isSquaredInputs()) {
            xSpeed = squareInput(xSpeed);
            ySpeed = squareInput(ySpeed);
            turnSpeed = squareInput(turnSpeed);
        }

        // Apply Field Oriented
        if (drive.isFieldOriented()) {
            double modifiedXSpeed = xSpeed;
            double modifiedYSpeed = ySpeed;

            Rotation2d heading = drive.getRotation2d();

            modifiedXSpeed = xSpeed * heading.getCos() + ySpeed * heading.getSin();
            modifiedYSpeed = -xSpeed * heading.getSin() + ySpeed * heading.getCos();

            xSpeed = modifiedXSpeed;
            ySpeed = modifiedYSpeed;
        }

        //Apply tipping
        double xTilt = drive.getRoll();
        double yTilt = drive.getPitch();

        switch (drive.getTippingState()) {
            case ANTI_TIP:
                if (Math.abs(xTilt) > Drive.TeleOpConstants.getAntiTippingXThresholdDegrees())
                    xSpeed = antiTipX.calculate(xSpeed, xTilt);
                if (Math.abs(yTilt) > Drive.TeleOpConstants.getAntiTippingYThresholdDegrees())
                    ySpeed = antiTipY.calculate(ySpeed, yTilt);
                break;
            case AUTO_BALANCE:
            case AUTO_BALANCE_ANTI_TIP:
                if (Math.abs(xTilt) > Drive.TeleOpConstants.getAutoBalanceXThreshold())
                    xSpeed = autoBalanceX.calculate(xSpeed, xTilt);
                if (Math.abs(yTilt) > Drive.TeleOpConstants.getAutoBalanceYThreshold())
                    ySpeed = autoBalanceX.calculate(ySpeed, yTilt);
                break;
            case NO_TIP_CORRECTION:
                break;            
        }

        // Apply deadband
        if (Math.abs(xSpeed) < DroidRageConstants.Gamepad.STICK_DEADZONE) xSpeed = 0;
        if (Math.abs(ySpeed) < DroidRageConstants.Gamepad.STICK_DEADZONE) ySpeed = 0;
        if (Math.abs(turnSpeed) < DroidRageConstants.Gamepad.STICK_DEADZONE) turnSpeed = 0;

        // Smooth driving and apply speed
        xSpeed = xLimiter.calculate(xSpeed) * SwerveModule.Constants.PHYSICAL_MAX_SPEED_METERS_PER_SECOND * drive.getTranslationalSpeed();
        ySpeed = yLimiter.calculate(ySpeed) * SwerveModule.Constants.PHYSICAL_MAX_SPEED_METERS_PER_SECOND * drive.getTranslationalSpeed();
        turnSpeed = turnLimiter.calculate(turnSpeed) * Drive.SwerveConstants.PHYSICAL_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND * drive.getAngularSpeed();

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turnSpeed);

        SwerveModuleState[] states = Drive.SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
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

    /**
     * Square magnitude of number while keeping the sign.
     */
    private double squareInput(double input) {
        return input * Math.abs(input);
    }
}
