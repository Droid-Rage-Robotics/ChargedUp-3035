package frc.robot.commands.Drive;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.SwerveModule;
import frc.robot.utilities.ComplexWidgetBuilder;
import frc.robot.DroidRageConstants;

public class SwerveDriveTeleop extends CommandBase {
    private final Drive drive;
    private final Supplier<Double> x, y, turn;
    private final SlewRateLimiter xLimiter, yLimiter, turnLimiter;
    private static final PIDController antiTipX = new PIDController(-0.1, 0, 0);
    private static final PIDController antiTipY = new PIDController(-0.1, 0, 0);
    private static final PIDController autoBalanceX = new PIDController(0.1, 0, 0);
    private static final PIDController autoBalanceY = new PIDController(0.1, 0, 0);

    private static final ComplexWidgetBuilder antiTipXWidget = new ComplexWidgetBuilder(
        antiTipX, "Anti Tip X PID Controller", Drive.class.getSimpleName()
    ).withWidget(BuiltInWidgets.kPIDController);

    private static final ComplexWidgetBuilder antiTipYWidget = new ComplexWidgetBuilder(
        antiTipX, "Anti Tip Y PID Controller", Drive.class.getSimpleName()
    ).withWidget(BuiltInWidgets.kPIDController);

    private static final ComplexWidgetBuilder autoBalanceXWidget = new ComplexWidgetBuilder(
        antiTipX, "Auto Balance X PID Controller", Drive.class.getSimpleName()
    ).withWidget(BuiltInWidgets.kPIDController);

    private static final ComplexWidgetBuilder autoBalanceYWidget = new ComplexWidgetBuilder(
        antiTipX, "Auto Balance Y PID Controller", Drive.class.getSimpleName()
    ).withWidget(BuiltInWidgets.kPIDController)
        .withSize(0, 2);

    private volatile double xSpeed, ySpeed, turnSpeed;
    public SwerveDriveTeleop(Drive drive,
            Supplier<Double> x, Supplier<Double> y, Supplier<Double> turn) {
        this.drive = drive;
        this.x = x;
        this.y = y;
        this.turn = turn;

        this.xLimiter = new SlewRateLimiter(Drive.TeleOpNumbers.MAX_ACCELERATION_UNITS_PER_SECOND.value.get());
        this.yLimiter = new SlewRateLimiter(Drive.TeleOpNumbers.MAX_ACCELERATION_UNITS_PER_SECOND.value.get());
        this.turnLimiter = new SlewRateLimiter(Drive.TeleOpNumbers.MAX_ANGULAR_ACCELERATION_UNITS_PER_SECOND.value.get());

        antiTipX.setTolerance(3);
        antiTipY.setTolerance(3);
        autoBalanceX.setTolerance(1);
        autoBalanceY.setTolerance(1);


        addRequirements(drive);
        antiTipXWidget.get();
        antiTipYWidget.get();
        autoBalanceXWidget.get();
        autoBalanceYWidget.get();
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        xSpeed = -y.get();
        ySpeed = x.get();
        turnSpeed = -turn.get();

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
                if (Math.abs(xTilt) > antiTipX.getPositionTolerance())
                    xSpeed = antiTipX.calculate(xSpeed, xTilt);
                if (Math.abs(yTilt) > antiTipY.getPositionTolerance())
                    ySpeed = antiTipY.calculate(ySpeed, yTilt);
                break;
            case AUTO_BALANCE:
            case AUTO_BALANCE_ANTI_TIP:
                if (Math.abs(xTilt) > autoBalanceX.getPositionTolerance())
                    xSpeed = autoBalanceX.calculate(xSpeed, xTilt);
                if (Math.abs(yTilt) > autoBalanceY.getPositionTolerance())
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
        xSpeed = 
            xLimiter.calculate(xSpeed) * 
            SwerveModule.Constants.PHYSICAL_MAX_SPEED_METERS_PER_SECOND * 
            drive.getTranslationalSpeed();
        ySpeed = 
            yLimiter.calculate(ySpeed) *
            SwerveModule.Constants.PHYSICAL_MAX_SPEED_METERS_PER_SECOND *
            drive.getTranslationalSpeed();
        turnSpeed = 
            turnLimiter.calculate(turnSpeed) * 
            Drive.SwerveConstants.PHYSICAL_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND * 
            drive.getAngularSpeed();

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turnSpeed);

        SwerveModuleState[] states = Drive.SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        drive.setModuleStates(states);
    }

    @Override
    public void end(boolean interrupted) {
        antiTipX.close();
        antiTipY.close();
        autoBalanceX.close();
        autoBalanceY.close();
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
