package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
    public static class Constants {
        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4);
        public static final double DRIVE_MOTOR_GEAR_RATIO = 1 / 6.75;
        public static final double TURN_MOTOR_GEAR_RATIO = 1 / 21.42;

        public static final double DRIVE_ENCODER_ROT_2_METER = DRIVE_MOTOR_GEAR_RATIO * Math.PI * WHEEL_DIAMETER_METERS;
        public static final double DRIVE_ENCODER_RPM_2_METER_PER_SEC = DRIVE_ENCODER_ROT_2_METER / 60;
        public static final double TICKS_PER_REVOLUTION = 4096;
        public static final double TURN_ENCODER_ROT_2_RAD = 2 * Math.PI / TICKS_PER_REVOLUTION;

        public static final double TURN_P = 0.5;

        public static final double PHYSICAL_MAX_SPEED_METERS_PER_SECOND = 4.47;
    }

    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;

    private final Supplier<Double> absoluteEncoderOffsetRad;

    private final CANCoder turnEncoder;

    private final PIDController turningPidController;

    public SwerveModule(int driveMotorId, int turnMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, Supplier<Double> absoluteEncoderOffsetRad, boolean absoluteEncoderReversed) {
        
        this.absoluteEncoderOffsetRad = absoluteEncoderOffsetRad;

        turnEncoder = new CANCoder(absoluteEncoderId);
        CANCoderConfiguration config = new CANCoderConfiguration();
        config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition; // TODO: change to absolute eventually
        config.sensorDirection = absoluteEncoderReversed;
        config.sensorCoefficient = Constants.TURN_ENCODER_ROT_2_RAD; // cancoder reads in radians
        config.unitString = "rad";
        config.sensorTimeBase = SensorTimeBase.PerSecond;
        turnEncoder.configAllSettings(config);

        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        turnMotor = new CANSparkMax(turnMotorId, MotorType.kBrushless);

        driveMotor.setInverted(driveMotorReversed);
        turnMotor.setInverted(turningMotorReversed);

        driveMotor.getEncoder().setPositionConversionFactor(Constants.DRIVE_ENCODER_ROT_2_METER);
        driveMotor.getEncoder().setVelocityConversionFactor(Constants.DRIVE_ENCODER_RPM_2_METER_PER_SEC);

        turningPidController = new PIDController(Constants.TURN_P, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetDriveEncoder();
    }

    public double getDrivePos() {
        return driveMotor.getEncoder().getPosition();
    }

    public double getTurningPosition() {
        return turnEncoder.getAbsolutePosition() + absoluteEncoderOffsetRad.get();
    }

    public double getDriveVelocity(){
        return driveMotor.getEncoder().getVelocity();
    }

    public double getTurningVelocity(){
        return turnEncoder.getVelocity();
    }

    public double getTurnEncoderRad() {
        return getTurningPosition();
    }
    
    public void resetDriveEncoder(){
        driveMotor.getEncoder().setPosition(0);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePos(), new Rotation2d(getTurningPosition()));
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / Constants.PHYSICAL_MAX_SPEED_METERS_PER_SECOND);
        turnMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
        SmartDashboard.putString("Swerve[" + turnEncoder.getDeviceID() + "] state", state.toString());
        SmartDashboard.putString("Swerve[" + turnMotor.getDeviceId() + "] state", state.toString());
    }

    public void stop(){
        driveMotor.set(0);
        turnMotor.set(0);
    }

    public void coastMode() {
        driveMotor.setIdleMode(IdleMode.kCoast);
        turnMotor.setIdleMode(IdleMode.kCoast);
    }
}
