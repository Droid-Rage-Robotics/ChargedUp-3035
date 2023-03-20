package frc.robot.subsystems.Drive;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.Pigeon2.AxisDirection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.ComplexWidgetBuilder;
import frc.robot.utilities.ShuffleboardValueEnum;
import frc.robot.utilities.ShuffleboardValue;

//Set Voltage instead of set Power
//Set them to 90 to 100%
public class Drive extends SubsystemBase {
    public enum Config implements ShuffleboardValueEnum<Double> {
        PHYSICAL_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND(2 * (2 * Math.PI)),
        TRACK_WIDTH(Units.inchesToMeters(20.75)),
        WHEEL_BASE(Units.inchesToMeters(23.75)),

        MAX_ACCELERATION_UNITS_PER_SECOND(10),
        MAX_ANGULAR_ACCELERATION_UNITS_PER_SECOND(10),

        MAX_SPEED_METERS_PER_SECOND(SwerveModule.Constants.PHYSICAL_MAX_SPEED_METERS_PER_SECOND / 4),
        MAX_ANGULAR_SPEED_RADIANS_PER_SECOND(PHYSICAL_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND.get() / 10),
        MAX_ACCELERATION_METERS_PER_SECOND_SQUARED(1),
        MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED(1), // 1 / 8 of a full rotation per second per second),
        TRANSLATIONAL_KP(1.9),//1.9-0.09
        THETA_KP(0.01),//Changed to 1 from 0.2 (Lucky) =0.4 0.08

        FRONT_LEFT_ABSOLUTE_ENCODER_OFFSET_RADIANS(-2.774966), //1.24
        FRONT_RIGHT_ABSOLUTE_ENCODER_OFFSET_RADIANS(-3.854886), //2.26
        BACK_LEFT_ABSOLUTE_ENCODER_OFFSET_RADIANS(-2.589354), //0.99^//TODO:CHeck
        BACK_RIGHT_ABSOLUTE_ENCODER_OFFSET_RADIANS(-3.334867), //1.74

        DEFAULT_HEADING_OFFSET(0),
        ;
        private final ShuffleboardValue<Double> shuffleboardValue;
        private Config(double value) {
            shuffleboardValue = ShuffleboardValue.create(value, Config.class.getSimpleName()+"/"+name(), Drive.class.getSimpleName()).build();
        }
        @Override 
        public ShuffleboardValue<Double> getShuffleboardValue() { return shuffleboardValue; }
    }
    public enum TeleOpOptions implements ShuffleboardValueEnum<Boolean> { 
        IS_FIELD_ORIENTED(true),
        IS_SQUARED_INPUTS(true),
        ;
        private final ShuffleboardValue<Boolean> shuffleboardValue;
        private TeleOpOptions(boolean value) {
            shuffleboardValue = ShuffleboardValue.create(value, TeleOpOptions.class.getSimpleName()+"/"+name(), Drive.class.getSimpleName()).build();
        } 
        @Override 
        public ShuffleboardValue<Boolean> getShuffleboardValue() { return shuffleboardValue; }
    }

    public enum Speed {
        TURBO(1, 1),
        NORMAL(1, 1),
        SLOW(0.15, 0.15),
        SUPER_SLOW(0.05, 0.05),
        ;
        private final ShuffleboardValue<Double> shuffleboardTranslationalValue;
        private final ShuffleboardValue<Double> shuffleboardAngularValue;
        private Speed(double translationalSpeed, double angularSpeed) {
            shuffleboardTranslationalValue = ShuffleboardValue.create(translationalSpeed, Speed.class.getSimpleName()+"/"+name()+": Translational Speed", Drive.class.getSimpleName())
                .withSize(3, 3)
                .build();
            shuffleboardAngularValue = ShuffleboardValue.create(translationalSpeed, Speed.class.getSimpleName()+"/"+name()+": Angular Speed", Drive.class.getSimpleName())
                .withSize(3, 3)
                .build();
        }
        public double getTranslationalSpeed() {
            return shuffleboardTranslationalValue.get();
        }
        public double getAngularSpeed() {
            return shuffleboardAngularValue.get();
        }
    }

    public enum TippingState {
        NO_TIP_CORRECTION,
        ANTI_TIP,
        AUTO_BALANCE,
        AUTO_BALANCE_ANTI_TIP,
        ;
    }

    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(-Config.WHEEL_BASE.get() / 2, Config.TRACK_WIDTH.get() / 2),  // Front Left --
        new Translation2d(-Config.WHEEL_BASE.get() / 2, -Config.TRACK_WIDTH.get() / 2),   // Front Right +-
        new Translation2d(Config.WHEEL_BASE.get() / 2, Config.TRACK_WIDTH.get() / 2), // Back Left -+
        new Translation2d(Config.WHEEL_BASE.get() / 2, -Config.TRACK_WIDTH.get() / 2)   // Back Right ++
    );

    private final SwerveModule frontLeft = new SwerveModule(
        2,
        1,

        false, 
        false,

        11, 
        Config.FRONT_LEFT_ABSOLUTE_ENCODER_OFFSET_RADIANS::get,
        true
    );
    private final SwerveModule frontRight = new SwerveModule(
        4,
        3,

        false, 
        false,

        12, 
        Config.FRONT_RIGHT_ABSOLUTE_ENCODER_OFFSET_RADIANS::get,
        true
    );
    private final SwerveModule backLeft = new SwerveModule(
        8,
        7,

        false, 
        false,

        14, 
        Config.BACK_LEFT_ABSOLUTE_ENCODER_OFFSET_RADIANS::get,
        true
    );
    private final SwerveModule backRight = new SwerveModule(
        5,
        6,

        false, 
        false,

        13, 
        Config.BACK_RIGHT_ABSOLUTE_ENCODER_OFFSET_RADIANS::get,
        true
    );
    private final SwerveModule[] swerveModules = { frontLeft, frontRight, backLeft, backRight };

    private final Pigeon2 pigeon2 = new Pigeon2(15);

    private final SwerveDriveOdometry odometery = new SwerveDriveOdometry (
        DRIVE_KINEMATICS, 
        new Rotation2d(0), 
        getModulePositions()
    );

    private volatile Speed speed = Speed.NORMAL;
    private volatile TippingState tippingState = TippingState.NO_TIP_CORRECTION;

    private final ShuffleboardValue<String> tippingStateWriter = ShuffleboardValue.create(tippingState.name(), "Tipping State", Drive.class.getSimpleName()).build();
    private final ShuffleboardValue<String> speedStateWriter = ShuffleboardValue.create(speed.name(), "Speed/State", Drive.class.getSimpleName()).build();
    
    private final ShuffleboardValue<Double> headingWriter = ShuffleboardValue.create(0.0, "Gyro/Heading (Degrees)", Drive.class.getSimpleName()).build();
    private final ShuffleboardValue<Double> rollWriter = ShuffleboardValue.create(0.0, "Gyro/Roll (Degrees)", Drive.class.getSimpleName()).build();
    private final ShuffleboardValue<Double> pitchWriter = ShuffleboardValue.create(0.0, "Gyro/Pitch (Degrees)", Drive.class.getSimpleName()).build();
    private final ShuffleboardValue<String> locationWriter = ShuffleboardValue.create("", "Robot Location", Drive.class.getSimpleName()).build();

    private final ShuffleboardValue<Double> frontLeftTurnPositionWriter = ShuffleboardValue.create(0.0, "Swerve Modules/Front Left/Turn Position (Radians)", Drive.class.getSimpleName()).build();
    private final ShuffleboardValue<Double> frontLeftDriveDistanceWriter = ShuffleboardValue.create(0.0, "Swerve Modules/Front Left/Drive Position (Radians)", Drive.class.getSimpleName()).build();

    private final ShuffleboardValue<Double> frontRightTurnPositionWriter = ShuffleboardValue.create(0.0, "Swerve Modules/Front Right/Turn Position (Radians)", Drive.class.getSimpleName()).build();
    private final ShuffleboardValue<Double> frontRightDriveDistanceWriter = ShuffleboardValue.create(0.0, "Swerve Modules/Front Right/Drive Position (Radians)", Drive.class.getSimpleName()).build();
    
    private final ShuffleboardValue<Double> backLeftTurnPositionWriter = ShuffleboardValue.create(0.0, "Swerve Modules/Back Left/Turn Position (Radians)", Drive.class.getSimpleName()).build();
    private final ShuffleboardValue<Double> backLeftDriveDistanceWriter = ShuffleboardValue.create(0.0, "Swerve Modules/Back Left/Drive Position (Radians)", Drive.class.getSimpleName()).build();

    private final ShuffleboardValue<Double> backRightTurnPositionWriter = ShuffleboardValue.create(0.0, "Swerve Modules/Back Right/Turn Position (Radians)", Drive.class.getSimpleName()).build();
    private final ShuffleboardValue<Double> backRightDriveDistanceWriter = ShuffleboardValue.create(0.0, "Swerve Modules/Back Right/Drive Position (Radians)", Drive.class.getSimpleName()).build();

    private final ShuffleboardValue<Boolean> isEnabled = ShuffleboardValue.create(true, "Is Drive Enabled", Drive.class.getSimpleName()) 
        .withWidget(BuiltInWidgets.kToggleSwitch)
        .build();

    private boolean isBreakMode = false;

    private final Field2d field2d = new Field2d();

    public Drive() {
        //TODO: Make sure IMU RESETS
        for (SwerveModule swerveModule: swerveModules) {
            swerveModule.coastMode();
        }

        pigeon2.configMountPose(AxisDirection.NegativeX, AxisDirection.PositiveZ);
        resetOffset();

        ComplexWidgetBuilder.create(field2d, "Field", Drive.class.getSimpleName());
    }

    
    @Override
    public void periodic() {
        odometery.update(
            getRotation2d(),
            getModulePositions()
        );

        field2d.setRobotPose(getPose());

        headingWriter.set(getHeading());
        rollWriter.set(getRoll());
        pitchWriter.set(getPitch());
        locationWriter.set(getPose().getTranslation().toString());


        // frontLeftTurnAbsolutePositionWriter.set(frontLeft.getTurnEncoderRad());
        frontLeftTurnPositionWriter.set(frontLeft.getTurningPosition());
        frontLeftDriveDistanceWriter.set(frontLeft.getDrivePos());

        // frontRightTurnAbsolutePositionWriter.set(frontRight.getTurnEncoderRad());
        frontRightTurnPositionWriter.set(frontRight.getTurningPosition());
        frontRightDriveDistanceWriter.set(frontRight.getDrivePos());

        // backLeftTurnAbsolutePositionWriter.set(backLeft.getTurnEncoderRad());
        backLeftTurnPositionWriter.set(backLeft.getTurningPosition());
        backLeftDriveDistanceWriter.set(backLeft.getDrivePos());

        // backRightTurnAbsolutePositionWriter.set(backRight.getTurnEncoderRad());
        backRightTurnPositionWriter.set(backRight.getTurningPosition());
        backRightDriveDistanceWriter.set(backRight.getDrivePos());

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
        return Math.IEEEremainder(-pigeon2.getYaw(), 360);
    }

    public double getPitch() {
        return Math.IEEEremainder(pigeon2.getPitch(), 360);
    }

    public double getRoll() {
        return Math.IEEEremainder(pigeon2.getRoll(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());//TODO:Make
    }

    public Pose2d getPose() {
        return odometery.getPoseMeters();
    }

    public boolean isFieldOriented() {
        return TeleOpOptions.IS_FIELD_ORIENTED.shuffleboardValue.get();
    }

    public boolean isSquaredInputs() {
        return TeleOpOptions.IS_SQUARED_INPUTS.shuffleboardValue.get();
    }

    public double getTranslationalSpeed() {
        return speed.getTranslationalSpeed();
    }

    public double getAngularSpeed() {
        return speed.getAngularSpeed();
    }

    // public double getHeadingOffset() {
    //     return SwerveConfig.HEADING_OFFSET.value.get();
    // }

    public void resetOdometry(Pose2d pose) {
        odometery.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    public void drive(double xSpeed, double ySpeed, double turnSpeed) {
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turnSpeed);

        SwerveModuleState[] states = Drive.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

        setModuleStates(states);
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

    public void setFeedforwardModuleStates(SwerveModuleState[] states) {
        if (!isEnabled.get()) return;
        SwerveDriveKinematics.desaturateWheelSpeeds(
            states, 
            SwerveModule.Constants.PHYSICAL_MAX_SPEED_METERS_PER_SECOND
        );

        for (int i = 0; i < 4; i++) {
            swerveModules[i].setFeedforwardState(states[i]);
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
        return setSpeed(Speed.SUPER_SLOW);
    }


    public CommandBase resetEncoders() {
        return runOnce(() -> {
            for (SwerveModule swerveModule: swerveModules) {
                swerveModule.resetDriveEncoder();
                // pigeon2.setYaw(getAngularSpeed())
            }
        });
    }

    // public CommandBase resetHeading() {
    //     return runOnce(() -> SwerveConfig.HEADING_OFFSET.value.set(
    //             getHeadingOffset() + getHeading()
    //         )
    //     );
    // }

    // public CommandBase resetHeading(double target) {
    //     return runOnce(() -> SwerveConfig.HEADING_OFFSET.value.set(
    //             target + getHeading()
    //         )
    //     );
    // }
    private void setOffset(double degrees) {
        pigeon2.setYaw(degrees);
    }

    public CommandBase setOffsetCommand(double degrees) {
        return runOnce(() -> setOffset(degrees));
    }

    private void resetOffset() {
        setOffset(Config.DEFAULT_HEADING_OFFSET.get());
    }

    public CommandBase resetOffsetCommand() {
        return runOnce(this::resetOffset);
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
        return runOnce(() -> TeleOpOptions.IS_FIELD_ORIENTED.shuffleboardValue.set(
                !isFieldOriented()
            )
        );
    }

    public CommandBase toggleSquareInputs() {
        return runOnce(() -> TeleOpOptions.IS_SQUARED_INPUTS.shuffleboardValue.set(
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
            // switch (tipp ingState) {
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
            Config.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND.get(),
            Config.MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED.get());
    }
}
