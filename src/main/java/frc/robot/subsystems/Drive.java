package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.Pigeon2.AxisDirection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.MutableBoolean;
import frc.robot.utilities.MutableDouble;
import frc.robot.utilities.SimpleWidgetBuilder;
import frc.robot.utilities.WriteOnlyDouble;
import frc.robot.utilities.WriteOnlyString;

public class Drive extends SubsystemBase {
    public enum TeleOpNumbers {
        MAX_ACCELERATION_UNITS_PER_SECOND(10),
        MAX_ANGULAR_ACCELERATION_UNITS_PER_SECOND(10),
        ;
        public final MutableDouble value;
        private TeleOpNumbers(double value) {
            this.value = new MutableDouble(value, TeleOpNumbers.class.getSimpleName()+"/"+name(), Drive.class.getSimpleName());
        }
    }
    public enum TeleOpOptions { 
        IS_FIELD_ORIENTED(true),
        IS_SQUARED_INPUTS(true),
        ;

        public final MutableBoolean value;
        private TeleOpOptions(boolean value) {
            this.value = new MutableBoolean(value, TeleOpOptions.class.getSimpleName()+"/"+name(), Drive.class.getSimpleName());
        }
    }
    
    public enum AutoConfig {
        MAX_SPEED_METERS_PER_SECOND(SwerveModule.Constants.PHYSICAL_MAX_SPEED_METERS_PER_SECOND / 4),
        MAX_ANGULAR_SPEED_RADIANS_PER_SECOND(SwerveConstants.PHYSICAL_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND / 10),
        MAX_ACCELERATION_METERS_PER_SECOND_SQUARED(3),
        MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED(Math.PI / 4), // 1 / 8 of a full rotation per second per second),
        TRANSLATIONAL_KP(1.5), // this could probably be about 2.29
        THETA_KP(0.005),
        ;
        public final MutableDouble value;
        private AutoConfig(double value) {
            this.value = new MutableDouble(value, AutoConfig.class.getSimpleName()+"/"+name(), Drive.class.getSimpleName());
        }
    }

    public enum SwerveConfig {
        FRONT_LEFT_ABSOLUTE_ENCODER_OFFSET_RADIANS(-2.54-0.26), //1.24
        FRONT_RIGHT_ABSOLUTE_ENCODER_OFFSET_RADIANS(-3.76), //2.26
        BACK_LEFT_ABSOLUTE_ENCODER_OFFSET_RADIANS(-2.54), //0.99
        BACK_RIGHT_ABSOLUTE_ENCODER_OFFSET_RADIANS(-3.27), //1.74

        HEADING_OFFSET(90)
        ;
        public final MutableDouble value;
        private SwerveConfig(double value) {
            this.value = new MutableDouble(value, SwerveConfig.class.getSimpleName()+"/"+name(), Drive.class.getSimpleName());
        }
    }

    public static class SwerveConstants {
        public static final double PHYSICAL_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 2 * (2 * Math.PI);
        public static final double TRACK_WIDTH = Units.inchesToMeters(20.75);
        public static final double WHEEL_BASE = Units.inchesToMeters(23.75);
        public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),  // Front Left --
            new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2),   // Front Right +-
            new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2), // Back Left -+
            new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2)   // Back Right ++
        );
    }

    private enum Speed {
        TURBO(1, 1), //TODO find what feels best 
        NORMAL(1, 1),
        SLOW(0.15, 0.15),
        SUPERSLOW(0.05, 0.05), //unused
        ;

        private final MutableDouble translationalSpeed;
        private final MutableDouble angularSpeed;

        private Speed(double translationalSpeed, double angularSpeed) {
            this.translationalSpeed = new MutableDouble(translationalSpeed, Speed.class.getSimpleName()+"/"+name()+"/Translational Speed", Drive.class.getSimpleName());
            this.angularSpeed = new MutableDouble(angularSpeed, Speed.class.getSimpleName()+"/"+name()+"/Angular Speed", Drive.class.getSimpleName());
        }
    }

    public enum TippingState {
        NO_TIP_CORRECTION,
        ANTI_TIP,
        AUTO_BALANCE,
        AUTO_BALANCE_ANTI_TIP,
        ;
    }

    private final SwerveModule frontLeft = new SwerveModule(
        2,
        1,

        false, 
        false,

        11, 
        SwerveConfig.FRONT_LEFT_ABSOLUTE_ENCODER_OFFSET_RADIANS.value::get,
        true
    );
    private final SwerveModule frontRight = new SwerveModule(
        4,
        3,

        false, 
        false,

        12, 
        SwerveConfig.FRONT_RIGHT_ABSOLUTE_ENCODER_OFFSET_RADIANS.value::get,
        true
    );
    private final SwerveModule backLeft = new SwerveModule(
        8,
        7,

        false, 
        false,

        14, 
        SwerveConfig.BACK_LEFT_ABSOLUTE_ENCODER_OFFSET_RADIANS.value::get,
        true
    );
    private final SwerveModule backRight = new SwerveModule(
        6,
        5,

        false, 
        false,

        13, 
        SwerveConfig.BACK_RIGHT_ABSOLUTE_ENCODER_OFFSET_RADIANS.value::get,
        true
    );
    private final SwerveModule[] swerveModules = { frontLeft, frontRight, backLeft, backRight };

    private final Pigeon2 pigeon2 = new Pigeon2(15);

    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry (
        SwerveConstants.DRIVE_KINEMATICS, 
        new Rotation2d(0), 
        getModulePositions()
    );

    private volatile Speed speed = Speed.NORMAL;
    private volatile TippingState tippingState = TippingState.NO_TIP_CORRECTION;

    private final WriteOnlyString tippingStateWriter = new WriteOnlyString(tippingState.name(), "Tipping State", Drive.class.getSimpleName());
    private final WriteOnlyString speedStateWriter = new WriteOnlyString(speed.name(), "Speed State", Drive.class.getSimpleName());
    
    private final WriteOnlyDouble headingWriter = new WriteOnlyDouble(0, "Gyro/Heading (Degrees)", Drive.class.getSimpleName());
    private final WriteOnlyDouble rollWriter = new WriteOnlyDouble(0, "Gyro/Roll (Degrees)", Drive.class.getSimpleName());
    private final WriteOnlyDouble pitchWriter = new WriteOnlyDouble(0, "Gyro/Pitch (Degrees)", Drive.class.getSimpleName());
    private final WriteOnlyString locationWriter = new WriteOnlyString("", "Robot Location", Drive.class.getSimpleName());

    private final WriteOnlyDouble frontLeftTurnPositionWriter = new WriteOnlyDouble(0, "Swerve Modules/Front Left/Turn Position (Radians)", Drive.class.getSimpleName());
    private final WriteOnlyDouble frontLeftTurnAbsolutePositionWriter = new WriteOnlyDouble(0, "Swerve Modules/Front Left/Absolute Position (Radians)", Drive.class.getSimpleName());
    private final WriteOnlyDouble frontLeftDriveDistanceWriter = new WriteOnlyDouble(0, "Swerve Modules/Front Left/Drive Position (Radians)", Drive.class.getSimpleName());

    private final WriteOnlyDouble frontRightTurnPositionWriter = new WriteOnlyDouble(0, "Swerve Modules/Front Right/Turn Position (Radians)", Drive.class.getSimpleName());
    private final WriteOnlyDouble frontRightTurnAbsolutePositionWriter = new WriteOnlyDouble(0, "Swerve Modules/Front Right/Absolute Position (Radians)", Drive.class.getSimpleName());
    private final WriteOnlyDouble frontRightDriveDistanceWriter = new WriteOnlyDouble(0, "Swerve Modules/Front Right/Drive Position (Radians)", Drive.class.getSimpleName());
    
    private final WriteOnlyDouble backLeftTurnPositionWriter = new WriteOnlyDouble(0, "Swerve Modules/Back Left/Turn Position (Radians)", Drive.class.getSimpleName());
    private final WriteOnlyDouble backLeftTurnAbsolutePositionWriter = new WriteOnlyDouble(0, "Swerve Modules/Back Left/Absolute Position (Radians)", Drive.class.getSimpleName());
    private final WriteOnlyDouble backLeftDriveDistanceWriter = new WriteOnlyDouble(0, "Swerve Modules/Back Left/Drive Position (Radians)", Drive.class.getSimpleName());

    private final WriteOnlyDouble backRightTurnPositionWriter = new WriteOnlyDouble(0, "Swerve Modules/Back Right/Turn Position (Radians)", Drive.class.getSimpleName());
    private final WriteOnlyDouble backRightTurnAbsolutePositionWriter = new WriteOnlyDouble(0, "Swerve Modules/Back Right/Absolute Position (Radians)", Drive.class.getSimpleName());
    private final WriteOnlyDouble backRightDriveDistanceWriter = new WriteOnlyDouble(0, "Swerve Modules/Back Right/Drive Position (Radians)", Drive.class.getSimpleName());

    private final MutableBoolean isEnabled = new SimpleWidgetBuilder<Boolean>(true, "Is Drive Enabled", Drive.class.getSimpleName())
        .withWidget(BuiltInWidgets.kToggleSwitch)
        .buildMutableBoolean();

        private boolean isBreakMode = false;

    public Drive() {
        //TODO: Make sure IMU RESETS
        for (SwerveModule swerveModule: swerveModules) {
            swerveModule.coastMode();
        }

        pigeon2.configMountPose(AxisDirection.NegativeX, AxisDirection.PositiveZ);
    }

    @Override
    public void periodic() {
        odometer.update(
            getRotation2d(),
            getModulePositions()
        );

        headingWriter.set(getHeading());
        rollWriter.set(getRoll());
        pitchWriter.set(getPitch());
        locationWriter.set(getPose().getTranslation().toString());


        // frontLeftTurnAbsolutePositionWriter.set(frontLeft.getTurnEncoderRad());
        frontLeftTurnPositionWriter.set(frontLeft.getTurningPosition());
        frontLeftDriveDistanceWriter.set(frontLeft.getDrivePos());

        // frontRightTurnAbsolutePositionWriter.set(frontRight.getTurnEncoderRad());
        frontRightTurnPositionWriter.set(frontRight.getTurningPosition());
        frontLeftDriveDistanceWriter.set(frontRight.getDrivePos());

        // backLeftTurnAbsolutePositionWriter.set(backLeft.getTurnEncoderRad());
        backLeftTurnPositionWriter.set(backLeft.getTurningPosition());
        frontLeftDriveDistanceWriter.set(backLeft.getDrivePos());

        // backRightTurnAbsolutePositionWriter.set(backRight.getTurnEncoderRad());
        backRightTurnPositionWriter.set(backRight.getTurningPosition());
        frontLeftDriveDistanceWriter.set(backRight.getDrivePos());

    }

    @Override
    public void simulationPeriodic() {
        periodic();
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        };
    }

    public TippingState getTippingState() {
        return tippingState;
    }

    public double getHeading() {
        return Math.IEEEremainder(-pigeon2.getYaw(), 360) - getHeadingOffset();
    }

    public double getPitch() {
        return Math.IEEEremainder(pigeon2.getPitch(), 360);
    }

    public double getRoll() {
        return Math.IEEEremainder(pigeon2.getRoll(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public boolean isFieldOriented() {
        return TeleOpOptions.IS_FIELD_ORIENTED.value.get();
    }

    public boolean isSquaredInputs() {
        return TeleOpOptions.IS_SQUARED_INPUTS.value.get();
    }

    public double getTranslationalSpeed() {
        return speed.translationalSpeed.get();
    }

    public double getAngularSpeed() {
        return speed.angularSpeed.get();
    }

    public double getHeadingOffset() {
        return SwerveConfig.HEADING_OFFSET.value.get();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    public void setModuleStates(SwerveModuleState[] states) {
        if (!isEnabled.get()) return;
        SwerveDriveKinematics.desaturateWheelSpeeds(
            states, 
            SwerveModule.Constants.PHYSICAL_MAX_SPEED_METERS_PER_SECOND
        );

        for (int i = 0; i < 4; i++) {
            swerveModules[i].setState(states[i]);
        }
    }

    public void stop() {
        for (SwerveModule swerveModule: swerveModules) {
            swerveModule.stop();
        }
    }

    public void setTippingState(TippingState tippingState) {
        this.tippingState = tippingState;
        tippingStateWriter.set(tippingState.name());
    }

    private CommandBase setSpeed(Speed speed) {
        return runOnce(() -> {
            this.speed = speed;
            speedStateWriter.set(speed.name());
        });
    }

    public CommandBase setTurboSpeed() {
        return setSpeed(Speed.TURBO);
    }

    public CommandBase setNormalSpeed() {
        return setSpeed(Speed.NORMAL);
    }

    public CommandBase setSlowSpeed() {
        return setSpeed(Speed.SLOW);
    }

    public CommandBase setSupserSlowSpeed() {
        return setSpeed(Speed.SUPERSLOW);
    }


    public CommandBase resetEncoders() {
        return runOnce(() -> {
            for (SwerveModule swerveModule: swerveModules) {
                swerveModule.resetDriveEncoder();
            }
        });
    }

    public CommandBase resetHeading() {
        return runOnce(() -> SwerveConfig.HEADING_OFFSET.value.set(
                getHeadingOffset() + getHeading()
            )
        );
    }

    public CommandBase resetHeading(double target) {
        return runOnce(() -> SwerveConfig.HEADING_OFFSET.value.set(
                target + getHeading()
            )
        );
    }

    private void coastMode() {
        for (SwerveModule swerveModule: swerveModules) {
            swerveModule.coastMode();
        }
    }

    private void breakMode() {
        for (SwerveModule swerveModule: swerveModules) {
            swerveModule.breakMode();
        }
    }

    public CommandBase toggleBreakMode() {
        return runOnce(() -> {
            if (isBreakMode) {
                coastMode();
                isBreakMode = false;
            } else {
                breakMode();
                isBreakMode = true;
            }
        });
    }


    public CommandBase toggleFieldOriented() {
        return runOnce(() -> TeleOpOptions.IS_FIELD_ORIENTED.value.set(
                !isFieldOriented()
            )
        );
    }

    public CommandBase toggleSquareInputs() {
        return runOnce(() -> TeleOpOptions.IS_SQUARED_INPUTS.value.set(
                !isSquaredInputs()
            )
        );
    }

    public CommandBase toggleAntiTipping() {
        // TODO: if you can upgrade java version to 17 in gradle and it works, you could simplify this switch statement using switch expression
        return runOnce(() -> setTippingState(
            TippingState.NO_TIP_CORRECTION
            // switch (tippingState) {
            //     case ANTI_TIP -> TippingState.NO_TIP_CORRECTION;
            //     case AUTO_BALANCE -> TippingState.AUTO_BALANCE_ANTI_TIP;
            //     case AUTO_BALANCE_ANTI_TIP -> TippingState.AUTO_BALANCE;
            //     case NO_TIP_CORRECTION ->TippingState.ANTI_TIP;
            // }
        ));
    }

    public CommandBase toggleAutoBalance() {
        return runOnce(() -> setTippingState(
            TippingState.NO_TIP_CORRECTION
            // switch (tippingState) {
            //     case ANTI_TIP -> TippingState.AUTO_BALANCE_ANTI_TIP;
            //     case AUTO_BALANCE -> TippingState.NO_TIP_CORRECTION;
            //     case AUTO_BALANCE_ANTI_TIP -> TippingState.ANTI_TIP;
            //     case NO_TIP_CORRECTION ->TippingState.AUTO_BALANCE;
            // }
        ));
    }

    public CommandBase runStop() {
        return runOnce(this::stop);
    }

    public TrapezoidProfile.Constraints getThetaConstraints() { //TODO when do you use this? I think for turning
        return new TrapezoidProfile.Constraints(
            AutoConfig.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND.value.get(),
            AutoConfig.MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED.value.get());
    }
}