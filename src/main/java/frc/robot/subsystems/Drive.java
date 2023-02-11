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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DroidRageConstants;

public class Drive extends SubsystemBase {
    public static class TeleOpConstants {
        public static final double MAX_ACCELERATION_UNITS_PER_SECOND = 3;//TODO find what feels best
        public static final double MAX_ANGULAR_ACCELERATION_UINTS_PER_SECOND = 3;//TODO find what feels best

        private static final double ANTI_TIPPING_X_THRESHOLD_DEGREES = 10;//TODO
        private static final double ANTI_TIPPING_X_P = 0; //TODO this might be negative
        private static final double ANTI_TIPPING_X_D = 0; //TODO

        private static final double ANTI_TIPPING_Y_THRESHOLD_DEGREES = ANTI_TIPPING_X_THRESHOLD_DEGREES;//TODO
        private static final double ANTI_TIPPING_Y_P = ANTI_TIPPING_X_P; //TODO
        private static final double ANTI_TIPPING_Y_D = ANTI_TIPPING_X_D; //TODO

        private static final double AUTO_BALANCE_X_THRESHOLD_DEGREES = 0;//TODO
        private static final double AUTO_BALANCE_X_P = 0;//TODO this might be negative
        private static final double AUTO_BALANCE_X_D = 0;//TODO

        private static final double AUTO_BALANCE_Y_THRESHOLD_DEGREES = AUTO_BALANCE_X_THRESHOLD_DEGREES;//TODO
        private static final double AUTO_BALANCE_Y_P = AUTO_BALANCE_X_P;//TODO
        private static final double AUTO_BALANCE_Y_D = AUTO_BALANCE_X_D;//TODO

        public static double getAntiTippingXThresholdDegrees() {
            return DroidRageConstants.getNumber("Drive/TeleOp/ANTI_TIPPING_X_THRESHOLD_DEGREES", ANTI_TIPPING_X_THRESHOLD_DEGREES);
        }

        public static double getAntiTippingXP() {
            return DroidRageConstants.getNumber("Drive/TeleOp/ANTI_TIPPING_X_P", ANTI_TIPPING_X_P);
        }

        public static double getAntiTippingXD() {
            return DroidRageConstants.getNumber("Drive/TeleOp/ANTI_TIPPING_X_D", ANTI_TIPPING_X_D);
        }

        public static double getAntiTippingYThresholdDegrees() {
            return DroidRageConstants.getNumber("Drive/TeleOp/ANTI_TIPPING_Y_THRESHOLD_DEGREES", ANTI_TIPPING_Y_THRESHOLD_DEGREES);
        }

        public static double getAntiTippingYP() {
            return DroidRageConstants.getNumber("Drive/TeleOp/ANTI_TIPPING_Y_P", ANTI_TIPPING_Y_P);
        }

        public static double getAntiTippingYD() {
            return DroidRageConstants.getNumber("Drive/TeleOp/ANTI_TIPPING_Y_D", ANTI_TIPPING_Y_D);
        }

        public static double getAutoBalanceXThreshold() {
            return DroidRageConstants.getNumber("Drive/TeleOp/AUTO_BALANCE_X_THRESHOLD_DEGREES", AUTO_BALANCE_X_THRESHOLD_DEGREES);
        }

        public static double getAutoBalanceXP() {
            return DroidRageConstants.getNumber("Drive/TeleOp/AUTO_BALANCE_X_P", AUTO_BALANCE_X_P);
        }

        public static double getAutoBalanceXD() {
            return DroidRageConstants.getNumber("Drive/TeleOp/AUTO_BALANCE_X_D", AUTO_BALANCE_X_D);
        }

        public static double getAutoBalanceYThreshold() {
            return DroidRageConstants.getNumber("Drive/TeleOp/UTO_BALANCE_Y_THRESHOLD_DEGREES", AUTO_BALANCE_Y_THRESHOLD_DEGREES);
        }

        public static double getAutoBalanceYP() {
            return DroidRageConstants.getNumber("Drive/TeleOp/AUTO_BALANCE_Y_P", AUTO_BALANCE_Y_P);
        }

        public static double getAutoBalanceYD() {
            return DroidRageConstants.getNumber("Drive/TeleOp/AUTO_BALANCE_Y_D", AUTO_BALANCE_Y_D);
        }
    }
    
    public static class AutoConstants {
        private static final double MAX_SPEED_METERS_PER_SECOND = 
            SwerveModule.Constants.PHYSICAL_MAX_SPEED_METERS_PER_SECOND / 4;

        private static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 
            SwerveConstants.PHYSICAL_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND / 10;

        private static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3; // 3 meters per second per second
        private static final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = Math.PI / 4; // 1 / 8 of a full rotation per second per second
        //TODO: these will change probably
        private static final double TRANSLATIONAL_KP = 1.5; // this could probably be about 2.29
        private static final double THETA_KP = 3;

        private static final TrapezoidProfile.Constraints THETA_CONSTRAINTS = 
            new TrapezoidProfile.Constraints(
                MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, 
                MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED
            );

        public static double getMaxSpeedMetersPerSecond() {
            return DroidRageConstants.getNumber("Drive/Auto/MAX_SPEED_METERS_PER_SECOND", MAX_SPEED_METERS_PER_SECOND);
        }

        public static double getMaxAngularSpeedRadiansPerSecond() {
            return DroidRageConstants.getNumber("Drive/Auto/MAX_ANGULAR_SPEED_RADIANS_PER_SECOND", MAX_ANGULAR_SPEED_RADIANS_PER_SECOND);
        }

        public static double getMaxAccelerationMetersPerSecondSquared() {
            return DroidRageConstants.getNumber("Drive/Auto/MAX_ACCELERATION_METERS_PER_SECOND_SQUARED", MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
        }

        public static double getMaxAngularAccelerationRadiansPerSecondSquared() {
            return DroidRageConstants.getNumber("Drive/Auto/MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED", MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED);
        }

        public static double getTranslationalKp() {
            return DroidRageConstants.getNumber("Drive/Auto/TRANSLATIONAL_KP", TRANSLATIONAL_KP);
        }

        public static double getThetaKp() {
            return DroidRageConstants.getNumber("Drive/Auto/THETA_KP", THETA_KP);
        }

        public static TrapezoidProfile.Constraints getThetaConstraints() {
            return new TrapezoidProfile.Constraints(
                getMaxAngularSpeedRadiansPerSecond(), 
                getMaxAngularAccelerationRadiansPerSecondSquared()
            );
        }
    }

    public static class SwerveConstants {
        public static final double PHYSICAL_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 2 * (2 * Math.PI); // 2 revolutions per second // I thought it was 4.41 before but no
              
        public static class FrontLeft {
            public static final int DRIVE_MOTOR_PORT = 2; //2
            public static final boolean DRIVE_MOTOR_REVERSED = true;

            public static final int TURN_MOTOR_PORT = 1; //1
            public static final boolean TURN_MOTOR_REVERSED = true;

            public static final int ABSOLUTE_ENCODER_PORT = 11;
            public static final double ABSOLUTE_ENCODER_OFFSET_RAD = 1.24;
            public static final boolean ABOSLUTE_ENCODER_REVERSED = !TURN_MOTOR_REVERSED;

            public static double getAbsoluteEncoderOffsetRad() {
                return DroidRageConstants.getNumber("Drive/Turn/Absolute Encoder Offset (Radians) Front Left", ABSOLUTE_ENCODER_OFFSET_RAD);
            }
        }

        public static class FrontRight {
            public static final int DRIVE_MOTOR_PORT = 4; // 4
            public static final boolean DRIVE_MOTOR_REVERSED = true;

            public static final int TURN_MOTOR_PORT = 3; //3
            public static final boolean TURN_MOTOR_REVERSED = true;

            public static final int ABSOLUTE_ENCODER_PORT = 12;
            private static final double ABSOLUTE_ENCODER_OFFSET_RAD = 2.26;
            public static final boolean ABOSLUTE_ENCODER_REVERSED = !TURN_MOTOR_REVERSED;

            public static double getAbsoluteEncoderOffsetRad() {
                return DroidRageConstants.getNumber("Drive/Turn/Absolute Encoder Offset (Radians) Front Right", ABSOLUTE_ENCODER_OFFSET_RAD);
            }
        }

        public static class BackLeft {
            public static final int DRIVE_MOTOR_PORT = 8; //8
            public static final boolean DRIVE_MOTOR_REVERSED = true;

            public static final int TURN_MOTOR_PORT = 7; //7
            public static final boolean TURN_MOTOR_REVERSED = true;

            public static final int ABSOLUTE_ENCODER_PORT = 14;
            private static final double ABSOLUTE_ENCODER_OFFSET_RAD = 0.99;
            public static final boolean ABOSLUTE_ENCODER_REVERSED = !TURN_MOTOR_REVERSED;

            public static double getAbsoluteEncoderOffsetRad() {
                return DroidRageConstants.getNumber("Drive/Turn/Absolute Encoder Offset (Radians) Back Left", ABSOLUTE_ENCODER_OFFSET_RAD);
            }
        }

        public static class BackRight {
            public static final int DRIVE_MOTOR_PORT = 6; //6
            public static final boolean DRIVE_MOTOR_REVERSED = true;

            public static final int TURN_MOTOR_PORT = 5; //5
            public static final boolean TURN_MOTOR_REVERSED = true;

            public static final int ABSOLUTE_ENCODER_PORT = 13;
            private static final double ABSOLUTE_ENCODER_OFFSET_RAD = 1.74;
            public static final boolean ABOSLUTE_ENCODER_REVERSED = !TURN_MOTOR_REVERSED;

            public static double getAbsoluteEncoderOffsetRad() {
                return DroidRageConstants.getNumber("Drive/Turn/Absolute Encoder Offset (Radians) Back Right", ABSOLUTE_ENCODER_OFFSET_RAD);
            }
        }

        public static final double TRACK_WIDTH = Units.inchesToMeters(20.75); 
        public static final double WHEEL_BASE = Units.inchesToMeters(23.75);
        public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),  // Front Left --
            new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2),   // Front Right +-
            new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2), // Back Left -+
            new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2)   // Back Right ++
        );
    }

    private enum Speed {
        TURBO(1, 1), //TODO find what feels best 
        NORMAL(0.5, 0.5),
        SLOW(0.25, 0.25),
        ;

        private final double translationalSpeed;
        private final double angularSpeed;

        private Speed(double translationalSpeed, double angularSpeed) {
            this.translationalSpeed = translationalSpeed;
            this.angularSpeed = angularSpeed;
        }
    }

    public enum TippingState {
        NO_TIP_CORRECTION,
        ANTI_TIP,
        AUTO_BALANCE,
        AUTO_BALANCE_ANTI_TIP
    }

    private final SwerveModule frontLeft = new SwerveModule(
        SwerveConstants.FrontLeft.DRIVE_MOTOR_PORT,
        SwerveConstants.FrontLeft.TURN_MOTOR_PORT,

        SwerveConstants.FrontLeft.DRIVE_MOTOR_REVERSED, 
        SwerveConstants.FrontLeft.TURN_MOTOR_REVERSED,

        SwerveConstants.FrontLeft.ABSOLUTE_ENCODER_PORT, 
        SwerveConstants.FrontLeft::getAbsoluteEncoderOffsetRad,
        SwerveConstants.FrontLeft.ABOSLUTE_ENCODER_REVERSED
    );
    private final SwerveModule frontRight = new SwerveModule(
        SwerveConstants.FrontRight.DRIVE_MOTOR_PORT,
        SwerveConstants.FrontRight.TURN_MOTOR_PORT,

        SwerveConstants.FrontRight.DRIVE_MOTOR_REVERSED, 
        SwerveConstants.FrontRight.TURN_MOTOR_REVERSED,

        SwerveConstants.FrontRight.ABSOLUTE_ENCODER_PORT, 
        SwerveConstants.FrontRight::getAbsoluteEncoderOffsetRad, 
        SwerveConstants.FrontRight.ABOSLUTE_ENCODER_REVERSED
    );
    private final SwerveModule backLeft = new SwerveModule(
        SwerveConstants.BackLeft.DRIVE_MOTOR_PORT,
        SwerveConstants.BackLeft.TURN_MOTOR_PORT,

        SwerveConstants.BackLeft.DRIVE_MOTOR_REVERSED, 
        SwerveConstants.BackLeft.TURN_MOTOR_REVERSED,

        SwerveConstants.BackLeft.ABSOLUTE_ENCODER_PORT, 
        SwerveConstants.BackLeft::getAbsoluteEncoderOffsetRad, 
        SwerveConstants.BackLeft.ABOSLUTE_ENCODER_REVERSED
    );
    private final SwerveModule backRight = new SwerveModule(
        SwerveConstants.BackRight.DRIVE_MOTOR_PORT,
        SwerveConstants.BackRight.TURN_MOTOR_PORT,
        

        SwerveConstants.BackRight.DRIVE_MOTOR_REVERSED, 
        SwerveConstants.BackRight.TURN_MOTOR_REVERSED,

        SwerveConstants.BackRight.ABSOLUTE_ENCODER_PORT, 
        SwerveConstants.BackRight::getAbsoluteEncoderOffsetRad, 
        SwerveConstants.BackRight.ABOSLUTE_ENCODER_REVERSED
    );
    private final SwerveModule[] swerveModules = { frontLeft, frontRight, backLeft, backRight };

    // private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    private final Pigeon2 pigeon2 = new Pigeon2(15);

    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry (
        SwerveConstants.DRIVE_KINEMATICS, 
        new Rotation2d(0), 
        getModulePositions()
    );

    private volatile Speed speed = Speed.NORMAL;
    private volatile TippingState tippingState = TippingState.ANTI_TIP;

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

        DroidRageConstants.putNumber("Drive/Robot heading (Degrees)", getHeading());
        DroidRageConstants.putString("Drive/Robot Location", getPose().getTranslation().toString());

        DroidRageConstants.putNumber("Drive/Turn/Position/Front Left (Radians)", frontLeft.getTurningPosition());
        DroidRageConstants.putNumber("Drive/Turn/Position/Front Right (Radians)", frontRight.getTurningPosition());
        DroidRageConstants.putNumber("Drive/Turn/Position/Back Left (Radians)", backLeft.getTurningPosition());
        DroidRageConstants.putNumber("Drive/Turn/Position/Back Right (Radians)", backRight.getTurningPosition());

        DroidRageConstants.putNumber("Drive/Turn/Absolute Position/Front Left (Radians)", frontLeft.getTurnEncoderRad());
        DroidRageConstants.putNumber("Drive/Turn/Absolute Position/Front Right (Radians)", frontRight.getTurnEncoderRad());
        DroidRageConstants.putNumber("Drive/Turn/Absolute Position/Back Left (Radians)", backLeft.getTurnEncoderRad());
        DroidRageConstants.putNumber("Drive/Turn/Absolute Position/Back Right (Radians)", backRight.getTurnEncoderRad());

        DroidRageConstants.putNumber("Drive/Drive/Distance/Front Left (Radians)", frontLeft.getDrivePos());
        DroidRageConstants.putNumber("Drive/Drive/Distance/Front Right (Radians)", frontRight.getDrivePos());
        DroidRageConstants.putNumber("Drive/Drive/Distance/Back Left (Radians)", backLeft.getDrivePos());
        DroidRageConstants.putNumber("Drive/Drive/Distance/Back Right (Radians)", backRight.getDrivePos());
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
        return DroidRageConstants.getBoolean("Drive/Field Oriented", true);
    }

    public boolean isSquaredInputs() {
        return DroidRageConstants.getBoolean("Drive/Squared Inputs", true);
    }

    public double getTranslationalSpeed() {
        return DroidRageConstants.getNumber("Drive/Speed Multiplier/Translational/"+ speed.name(), speed.translationalSpeed);
    }

    public double getAngularSpeed() {
        return DroidRageConstants.getNumber("Drive/Speed Multiplier/Angular/"+ speed.name(), speed.angularSpeed);
    }

    public double getHeadingOffset() {
        return DroidRageConstants.getNumber("Drive/Heading Offset", -90);
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    public void setModuleStates(SwerveModuleState[] states) {
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
        DroidRageConstants.putString("Drive/Tipping State", tippingState.name());
    }

    private CommandBase setSpeed(Speed speed) {
        return runOnce(() -> {
            this.speed = speed;
            DroidRageConstants.putString("Drive/Current Speed", speed.name());
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

    public CommandBase resetEncoders() {
        return runOnce(() -> {
            for (SwerveModule swerveModule: swerveModules) {
                swerveModule.resetDriveEncoder();
            }
        });
    }

    public CommandBase resetHeading() {
        return runOnce(() -> DroidRageConstants.putNumber(
                "Drive/Heading Offset", getHeadingOffset() + getHeading()
            ));
    }

    public CommandBase toggleFieldOriented() {
        return runOnce(() -> DroidRageConstants.putBoolean(
            "Drive/Field Oriented", !isFieldOriented()));
    }

    public CommandBase toggleSquareInputs() {
        return runOnce(() -> DroidRageConstants.putBoolean(
            "Drive/Squared Inputs", !isSquaredInputs()));
    }

    public CommandBase toggleAntiTipping() {
        // TODO: if you can upgrade java version to 17 in gradle and it works, you could simplify this switch statement using switch expression
        return runOnce(() -> {
            switch (tippingState) {
                case ANTI_TIP: 
                    tippingState = TippingState.NO_TIP_CORRECTION;
                    break;
                case AUTO_BALANCE:
                    tippingState = TippingState.AUTO_BALANCE_ANTI_TIP;
                    break;
                case AUTO_BALANCE_ANTI_TIP:
                    tippingState = TippingState.AUTO_BALANCE;
                    break;
                case NO_TIP_CORRECTION:
                    tippingState = TippingState.ANTI_TIP;
                    break;
            }
        });
    }

    public CommandBase toggleAutoBalance() {
        return runOnce(() -> {
            switch (tippingState) {
                case ANTI_TIP: 
                    tippingState = TippingState.AUTO_BALANCE_ANTI_TIP;
                    break;
                case AUTO_BALANCE:
                    tippingState = TippingState.NO_TIP_CORRECTION;
                    break;
                case AUTO_BALANCE_ANTI_TIP:
                    tippingState = TippingState.ANTI_TIP;
                    break;
                case NO_TIP_CORRECTION:
                    tippingState = TippingState.AUTO_BALANCE;
                    break;
            }
        });
    }

    public CommandBase runStop() {
        return runOnce(this::stop);
    }
}