package frc.robot.subsystems;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;

public class SwerveModule {
    public static class Constants {
        public static final double DRIVE_ENCODER_ROT_2_METER = 0;
        public static final double DRIVE_ENCODER_RPM_2_METER_PER_SEC = 0;
        public static final double TURN_ENCODER_ROT_2_RAD = 0;
        public static final double TURN_ENCODER_RPM_2_RAD_PER_SEC = 0;

        public static final double TURN_P = 0;

        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4);
        public static final double DRIVE_MOTOR_GEAR_RATIO = 1 / 6.75;

    }

    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;

    private final DroidRageEncoder turnEncoder;
    // turnEncoder.getAbosolutePosition();
    
    private final PIDController turningPidController;

    // private final AnalogInput 
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public SwerveModule(int driveMotorId, int turnMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffsetRad, boolean absoluteEncoderReversed) {
        
        this.absoluteEncoderOffsetRad = absoluteEncoderOffsetRad;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        turnEncoder = new DroidRageEncoder(absoluteEncoderId);

        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        turnMotor = new CANSparkMax(turnMotorId, MotorType.kBrushless);

        driveMotor.setInverted(driveMotorReversed);
        turnMotor.setInverted(turningMotorReversed);

        driveMotor.getEncoder().setPositionConversionFactor(Constants.DRIVE_ENCODER_ROT_2_METER);
        driveMotor.getEncoder().setVelocityConversionFactor(Constants.DRIVE_ENCODER_RPM_2_METER_PER_SEC);
        turnEncoder.setPositionConversionFactor(Constants.TURN_ENCODER_ROT_2_RAD);
        turnEncoder.setVelocityConversionFactor(Constants.TURN_ENCODER_RPM_2_RAD_PER_SEC);

        turningPidController = new PIDController(Constants.TURN_P, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    public double getDrivePos() {
        return driveMotor.getEncoder().getPosition();
    }

    public double getTurningPosition() {
        return turnEncoder.getPosition();
    }

    public double getDriveVelocity(){
        return driveMotor.getEncoder().getVelocity();
    }

    public double getTurningVelocity(){
        return turnEncoder.getVelocity();
    }

    public double getTurnEncoderRad() {
        double angle = turnEncoder.getVoltage() / RobotController.getVoltage5V();
        angle *= 2.0 * Math.PI;
        angle -= absoluteEncoderOffsetRad;
        if (absoluteEncoderReversed) angle = -angle;

        return angle;
    }
    
    public void resetEncoders(){
        driveMotor.getEncoder().setPosition(0);
        turnEncoder.setPosition(getTurnEncoderRad());
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setState(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, getState().angle);
        // driveMotor.set(state.speedMetersPerSecond / );
    }

    public void stop(){
        driveMotor.set(0);
        turnEncoder.setPosition(0);
    }
}
