package frc.robot.commands.drive;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.DroidRageConstants;
import frc.robot.subsystem.drive.Drive;
import frc.robot.subsystem.drive.DriveConstants;
import frc.robot.subsystem.drive.SwerveModule;

public class SwerveDriveTeleop extends CommandBase {
    private final Drive drive;
    private final Supplier<Double> x, y, turn;
    // private final SlewRateLimiter xLimiter, yLimiter, turnLimiter;
    // private static final PIDController autoBalanceY = new PIDController(0.006, 0, 0.0005);

    private volatile double xSpeed, ySpeed, turnSpeed;

    private Trigger lockTrigger;
    public SwerveDriveTeleop(Drive drive,
            Supplier<Double> x, Supplier<Double> y, Supplier<Double> turn, Trigger lockTrigger) {
        this.drive = drive;
        this.x = x;
        this.y = y;
        this.turn = turn;
        this.lockTrigger = lockTrigger;

        // this.xLimiter = new SlewRateLimiter(DriveConstants.Config.MAX_ACCELERATION_UNITS_PER_SECOND.get());
        // this.yLimiter = new SlewRateLimiter(DriveConstants.Config.MAX_ACCELERATION_UNITS_PER_SECOND.get());
        // this.turnLimiter = new SlewRateLimiter(DriveConstants.Config.MAX_ANGULAR_ACCELERATION_UNITS_PER_SECOND.get());



        addRequirements(drive);
    }

    @Override
    public void initialize() {    }

    @Override
    public void execute() {
        xSpeed = -y.get();
        ySpeed = -x.get();
        turnSpeed = -turn.get();

        // Square inputs
        if (drive.isSquaredInputs()) {
            xSpeed = DroidRageConstants.squareInput(xSpeed);
            ySpeed = DroidRageConstants.squareInput(ySpeed);
            turnSpeed = DroidRageConstants.squareInput(turnSpeed);
        }

        // Apply Field Oriented
        if (drive.isFieldOriented()) {
            double modifiedXSpeed = xSpeed;
            double modifiedYSpeed = ySpeed;

            
            Rotation2d heading = drive.getRotation2d();

            heading.rotateBy(
                Rotation2d.fromDegrees(
                    switch (DriverStation.getAlliance()) {
                        case Blue->00;
                        case Red->180;
                        case Invalid->0;
                    }
                )
            );

            modifiedXSpeed = xSpeed * heading.getCos() + ySpeed * heading.getSin();
            modifiedYSpeed = -xSpeed * heading.getSin() + ySpeed * heading.getCos();

            xSpeed = modifiedXSpeed;
            ySpeed = modifiedYSpeed;
        }

        // //Apply tipping
        // double xTilt = drive.getRoll();
        // double yTilt = drive.getPitch();

        // switch (drive.getTippingState()) {
        //     case ANTI_TIP:
        //         if (Math.abs(xTilt) > antiTipX.getPositionTolerance())
        //             xSpeed = antiTipX.calculate(xSpeed, xTilt);
        //         if (Math.abs(yTilt) > antiTipY.getPositionTolerance())
        //             ySpeed = antiTipY.calculate(ySpeed, yTilt);
        //         break;
        //     case AUTO_BALANCE:
        //     case AUTO_BALANCE_ANTI_TIP:
        //         if (Math.abs(xTilt) > autoBalanceX.getPositionTolerance())
        //             xSpeed = autoBalanceX.calculate(xSpeed, xTilt);
        //         // if (Math.abs(yTilt) > autoBalanceY.getPositionTolerance())
        //         //     ySpeed = autoBalanceY.calculate(ySpeed, yTilt);
        //         break;
        //     case NO_TIP_CORRECTION:
        //         break;            
        // }

        // Apply deadband
        if (Math.abs(xSpeed) < DroidRageConstants.Gamepad.DRIVER_STICK_DEADZONE) xSpeed = 0;
        if (Math.abs(ySpeed) < DroidRageConstants.Gamepad.DRIVER_STICK_DEADZONE) ySpeed = 0;
        if (Math.abs(turnSpeed) < DroidRageConstants.Gamepad.DRIVER_STICK_DEADZONE) turnSpeed = 0;

        // Smooth driving and apply speed
        xSpeed = 
            // xLimiter.calculate(xSpeed) * 
            xSpeed *
            SwerveModule.Constants.PHYSICAL_MAX_SPEED_METERS_PER_SECOND * 
            drive.getTranslationalSpeed();
        ySpeed = 
            // yLimiter.calculate(ySpeed) *
            ySpeed *
            SwerveModule.Constants.PHYSICAL_MAX_SPEED_METERS_PER_SECOND *
            drive.getTranslationalSpeed();
        turnSpeed = 
            // turnLimiter.calculate(turnSpeed) * 
            turnSpeed *
            DriveConstants.Config.PHYSICAL_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND.get() * 
            drive.getAngularSpeed();

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turnSpeed);

        SwerveModuleState[] states = Drive.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

        // if (lockTrigger.getAsBoolean()) {
        //     drive.setModuleStates(new SwerveModuleState[] {
        //         new SwerveModuleState(0.01, new Rotation2d(Math.PI / 4)),
        //         new SwerveModuleState(0.01, new Rotation2d(-Math.PI / 4)),
        //         new SwerveModuleState(0.01, new Rotation2d(-Math.PI / 4)),
        //         new SwerveModuleState(0.01, new Rotation2d(Math.PI / 4))
        //     });
        // }
        //TODO: Test
        drive.setModuleStates(states);
    }

    @Override
    public void end(boolean interrupted) {
        // autoBalanceY.close();
        drive.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
