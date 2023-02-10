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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
    public static class AutoConstants {
        public static final double MAX_SPEED_METERS_PER_SECOND = 
            SwerveModule.Constants.PHYSICAL_MAX_SPEED_METERS_PER_SECOND / 4;

        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 
            SwerveConstants.PHYSICAL_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND / 10;

        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3; // 3 meters per second per second
        public static final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = Math.PI / 4; // 1 / 8 of a full rotation per second per second
        public static final double TRANSLATIONAL_KP = 1.5;
        public static final double THETA_KP = 3;
        public static final TrapezoidProfile.Constraints THETA_CONSTRAINTS = 
            new TrapezoidProfile.Constraints(
                MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, 
                MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED
            );
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
        }

        public static class FrontRight {
            public static final int DRIVE_MOTOR_PORT = 4; // 4
            public static final boolean DRIVE_MOTOR_REVERSED = true;

            public static final int TURN_MOTOR_PORT = 3; //3
            public static final boolean TURN_MOTOR_REVERSED = true;

            public static final int ABSOLUTE_ENCODER_PORT = 12;
            public static final double ABSOLUTE_ENCODER_OFFSET_RAD = 2.26;
            public static final boolean ABOSLUTE_ENCODER_REVERSED = !TURN_MOTOR_REVERSED;
        }

        public static class BackLeft {
            public static final int DRIVE_MOTOR_PORT = 8; //8
            public static final boolean DRIVE_MOTOR_REVERSED = true;

            public static final int TURN_MOTOR_PORT = 7; //7
            public static final boolean TURN_MOTOR_REVERSED = true;

            public static final int ABSOLUTE_ENCODER_PORT = 14;
            public static final double ABSOLUTE_ENCODER_OFFSET_RAD = 0.99;
            public static final boolean ABOSLUTE_ENCODER_REVERSED = !TURN_MOTOR_REVERSED;
        }

        public static class BackRight {
            public static final int DRIVE_MOTOR_PORT = 6; //6
            public static final boolean DRIVE_MOTOR_REVERSED = true;

            public static final int TURN_MOTOR_PORT = 5; //5
            public static final boolean TURN_MOTOR_REVERSED = true;

            public static final int ABSOLUTE_ENCODER_PORT = 13;
            public static final double ABSOLUTE_ENCODER_OFFSET_RAD = 1.74;
            public static final boolean ABOSLUTE_ENCODER_REVERSED = !TURN_MOTOR_REVERSED;
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
        TURBO(1, 1),
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

    private final SwerveModule frontLeft = new SwerveModule(
        SwerveConstants.FrontLeft.DRIVE_MOTOR_PORT,
        SwerveConstants.FrontLeft.TURN_MOTOR_PORT,

        SwerveConstants.FrontLeft.DRIVE_MOTOR_REVERSED, 
        SwerveConstants.FrontLeft.TURN_MOTOR_REVERSED,

        SwerveConstants.FrontLeft.ABSOLUTE_ENCODER_PORT, 
        SwerveConstants.FrontLeft.ABSOLUTE_ENCODER_OFFSET_RAD, 
        SwerveConstants.FrontLeft.ABOSLUTE_ENCODER_REVERSED
    );
    private final SwerveModule frontRight = new SwerveModule(
        SwerveConstants.FrontRight.DRIVE_MOTOR_PORT,
        SwerveConstants.FrontRight.TURN_MOTOR_PORT,

        SwerveConstants.FrontRight.DRIVE_MOTOR_REVERSED, 
        SwerveConstants.FrontRight.TURN_MOTOR_REVERSED,

        SwerveConstants.FrontRight.ABSOLUTE_ENCODER_PORT, 
        SwerveConstants.FrontRight.ABSOLUTE_ENCODER_OFFSET_RAD, 
        SwerveConstants.FrontRight.ABOSLUTE_ENCODER_REVERSED
    );
    private final SwerveModule backLeft = new SwerveModule(
        SwerveConstants.BackLeft.DRIVE_MOTOR_PORT,
        SwerveConstants.BackLeft.TURN_MOTOR_PORT,

        SwerveConstants.BackLeft.DRIVE_MOTOR_REVERSED, 
        SwerveConstants.BackLeft.TURN_MOTOR_REVERSED,

        SwerveConstants.BackLeft.ABSOLUTE_ENCODER_PORT, 
        SwerveConstants.BackLeft.ABSOLUTE_ENCODER_OFFSET_RAD, 
        SwerveConstants.BackLeft.ABOSLUTE_ENCODER_REVERSED
    );
    private final SwerveModule backRight = new SwerveModule(
        SwerveConstants.BackRight.DRIVE_MOTOR_PORT,
        SwerveConstants.BackRight.TURN_MOTOR_PORT,
        

        SwerveConstants.BackRight.DRIVE_MOTOR_REVERSED, 
        SwerveConstants.BackRight.TURN_MOTOR_REVERSED,

        SwerveConstants.BackRight.ABSOLUTE_ENCODER_PORT, 
        SwerveConstants.BackRight.ABSOLUTE_ENCODER_OFFSET_RAD, 
        SwerveConstants.BackRight.ABOSLUTE_ENCODER_REVERSED
    );
    private final SwerveModule[] swerveModules = { frontLeft, frontRight, backLeft, backRight };

    // private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    private final Pigeon2 pigeon2 = new Pigeon2(15);

    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(
        SwerveConstants.DRIVE_KINEMATICS, 
        new Rotation2d(0), 
        getModulePositions()
    );

    private volatile boolean isFieldOriented = true;
    private volatile Speed speed = Speed.NORMAL;
    private volatile double headingOffset = -90;

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
        SmartDashboard.putNumber("Robot Heading Offset", headingOffset);
        SmartDashboard.putNumber("Robot heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
        
        SmartDashboard.putBoolean("Robot is Field Oriented", isFieldOriented);

        SmartDashboard.putNumber("Front Left Turn Distance", frontLeft.getTurningPosition());
        SmartDashboard.putNumber("Front Right Turn Distance", frontRight.getTurningPosition());
        SmartDashboard.putNumber("Back Left Turn Distance", backLeft.getTurningPosition());
        SmartDashboard.putNumber("Back Right Turn Distance", backRight.getTurningPosition());

        SmartDashboard.putNumber("Front Left Turn Angle Radians", frontLeft.getTurnEncoderRad());
        SmartDashboard.putNumber("Front Right Turn Angle Radians", frontRight.getTurnEncoderRad());
        SmartDashboard.putNumber("Back Left Turn Angle Radians", backLeft.getTurnEncoderRad());
        SmartDashboard.putNumber("Back Right Turn Angle Radians", backRight.getTurnEncoderRad());

        SmartDashboard.putNumber("Front Left Drive Distance Meters", frontLeft.getDrivePos());
        SmartDashboard.putNumber("Front Right Drive Distance Meters", frontRight.getDrivePos());
        SmartDashboard.putNumber("Back Left Drive Distance Meters", backLeft.getDrivePos());
        SmartDashboard.putNumber("Back Right Drive Distance Meters", backRight.getDrivePos());
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

    public double getHeading() {
        // return Math.IEEEremainder(gyro.getAngle(), 360);
        return Math.IEEEremainder(-pigeon2.getYaw(), 360) - headingOffset;
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    public boolean isFieldOriented() {
        return isFieldOriented;
    }

    public double getTranslationalSpeed() {
        return speed.translationalSpeed;
    }

    public double getAngularSpeed() {
        return speed.angularSpeed;
    }

    public double getHeadingOffset() {
        return headingOffset;
    }

    public void setModuleStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveModule.Constants.PHYSICAL_MAX_SPEED_METERS_PER_SECOND);
        for (int i = 0; i < 4; i++) {
            swerveModules[i].setState(states[i]);
        }
    }

    public void stop() {
        for (SwerveModule swerveModule: swerveModules) {
            swerveModule.stop();
        }
    }

    private CommandBase setSpeed(Speed speed) {
        return runOnce(() -> this.speed = speed);
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
        return runOnce(() -> headingOffset += getHeading());
    }

    public CommandBase toggleFieldOriented() {
        return runOnce(() -> isFieldOriented = !isFieldOriented);
    }

    public CommandBase runStop() {
        return runOnce(this::stop);
    }
    
}
