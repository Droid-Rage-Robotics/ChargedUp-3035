package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModuleWOCan {
    public static class Constants {
        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4);
        public static final double DRIVE_MOTOR_GEAR_RATIO = 1 / 6.75;
        public static final double TURN_MOTOR_GEAR_RATIO = 1 / 21.42;

        public static final double DRIVE_ENCODER_ROT_2_METER = DRIVE_MOTOR_GEAR_RATIO * Math.PI * WHEEL_DIAMETER_METERS;
        public static final double DRIVE_ENCODER_RPM_2_METER_PER_SEC = DRIVE_ENCODER_ROT_2_METER / 60;
        public static final double TURN_ENCODER_ROT_2_RAD = TURN_MOTOR_GEAR_RATIO * 2 * Math.PI;
        public static final double TURN_ENCODER_RPM_2_RAD_PER_SEC = TURN_ENCODER_ROT_2_RAD / 60;

        public static final double TURN_P = 0.5;

        public static final double PHYSICAL_MAX_SPEED_METERS_PER_SECOND = 4.4;//TODO:CHECk
        public static final double PHYSICAL_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 2 * (2 * Math.PI); // 2 revolutions per second
    }

    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;
    private final double resetPosition;
    private final PIDController turningPidController;

    public SwerveModuleWOCan(int driveMotorId, int turnMotorId, double resetPosition, boolean driveMotorReversed, boolean turningMotorReversed) {
        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        turnMotor = new CANSparkMax(turnMotorId, MotorType.kBrushless);

        this.resetPosition = resetPosition;

        driveMotor.setInverted(driveMotorReversed);
        turnMotor.setInverted(turningMotorReversed);

        driveMotor.getEncoder().setPositionConversionFactor(Constants.DRIVE_ENCODER_ROT_2_METER);
        driveMotor.getEncoder().setVelocityConversionFactor(Constants.DRIVE_ENCODER_RPM_2_METER_PER_SEC);

        turnMotor.getEncoder().setPositionConversionFactor(Constants.TURN_ENCODER_ROT_2_RAD);
        turnMotor.getEncoder().setVelocityConversionFactor(Constants.TURN_ENCODER_RPM_2_RAD_PER_SEC);

        turningPidController = new PIDController(Constants.TURN_P, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    public double getDrivePos() {
        return driveMotor.getEncoder().getPosition();
    }

    public double getTurningPosition() {
        return turnMotor.getEncoder().getPosition();
    }

    public double getDriveVelocity(){
        return driveMotor.getEncoder().getVelocity();
    }

    public double getTurningVelocity(){
        return turnMotor.getEncoder().getVelocity();
    }

    public double getTurnEncoderRad() {
        return getTurningPosition();
    }
    
    public void resetEncoders(){
        driveMotor.getEncoder().setPosition(0);
        turnMotor.getEncoder().setPosition(resetPosition);
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
        // SmartDashboard.putString("Swerve[" + turnEncoder.getDeviceID() + "] state", state.toString());
        SmartDashboard.putString("Swerve[" + turnMotor.getDeviceId() + "] state", state.toString());
    }

    public void stop(){
        driveMotor.set(0);
        turnMotor.set(0);
    }

    public void set(double power){
        driveMotor.set(power);
        // turnMotor.set(power);
    }

    public void breakMode() {
        driveMotor.setIdleMode(IdleMode.kCoast);
        turnMotor.setIdleMode(IdleMode.kCoast);
    }
}
