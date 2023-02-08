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
    public static class Constants {
        public static class Auto {
            public static final double MAX_SPEED_METERS_PER_SECOND = 
                SwerveModule.Constants.PHYSICAL_MAX_SPEED_METERS_PER_SECOND / 4;

            public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 
                SwerveModule.Constants.PHYSICAL_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND / 10;

            public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3; // 3 meters per second per second
            public static final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = Math.PI / 4; // 1 / 8 of a full rotation per second per second
            public static final double TRANSLATIONAL_KP = 1.5;
            public static final double THETA_KP = 3;
            public static final TrapezoidProfile.Constraints THETA_CONSTRAINTS = 
                new TrapezoidProfile.Constraints(
                    MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, 
                    MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED
                );

            public static final double MAX_ACCELERATION_METERS_PER_SECOND = 0; //TODO: CHECK THIS
        }
        public static class FrontLeft {
            public static final int DRIVE_MOTOR_PORT = 2;
            public static final boolean DRIVE_MOTOR_REVERSED = true;

            public static final int TURN_MOTOR_PORT = 1;
            public static final boolean TURN_MOTOR_REVERSED = false;// TODO: change

            public static final int ABSOLUTE_ENCODER_PORT = 11;// TODO: change
            public static final double ABSOLUTE_ENCODER_OFFSET_RAD = 0;// TODO: change
            public static final boolean ABOSLUTE_ENCODER_REVERSED = false;// TODO: change
        }

        public static class FrontRight {
            public static final int DRIVE_MOTOR_PORT = 4;
            public static final boolean DRIVE_MOTOR_REVERSED = true;

            public static final int TURN_MOTOR_PORT = 3;
            public static final boolean TURN_MOTOR_REVERSED = false;// TODO: change

            public static final int ABSOLUTE_ENCODER_PORT = 12;// TODO: change
            public static final double ABSOLUTE_ENCODER_OFFSET_RAD = 0;// TODO: change
            public static final boolean ABOSLUTE_ENCODER_REVERSED = false;// TODO: change
        }

        public static class BackLeft {
            public static final int DRIVE_MOTOR_PORT = 8;
            public static final boolean DRIVE_MOTOR_REVERSED = true;

            public static final int TURN_MOTOR_PORT = 7;
            public static final boolean TURN_MOTOR_REVERSED = false;// TODO: change

            public static final int ABSOLUTE_ENCODER_PORT = 14;// TODO: change
            public static final double ABSOLUTE_ENCODER_OFFSET_RAD = 0;// TODO: change
            public static final boolean ABOSLUTE_ENCODER_REVERSED = false;// TODO: change
        }

        public static class BackRight {
            public static final int DRIVE_MOTOR_PORT = 6;
            public static final boolean DRIVE_MOTOR_REVERSED = true;

            public static final int TURN_MOTOR_PORT = 5;
            public static final boolean TURN_MOTOR_REVERSED = false;// TODO: change

            public static final int ABSOLUTE_ENCODER_PORT = 13;// TODO: change
            public static final double ABSOLUTE_ENCODER_OFFSET_RAD = 0;// TODO: change
            public static final boolean ABOSLUTE_ENCODER_REVERSED = false;// TODO: change
        }

        public static final double TRACK_WIDTH = Units.inchesToMeters(20.75); 
        public static final double WHEEL_BASE = Units.inchesToMeters(23.75);
        public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2),  // Front Left
            new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),   // Front Right
            new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2), // Back Left
            new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2)   // Back Right
        );
    }

    private final SwerveModule frontLeft = new SwerveModule(
        Constants.FrontLeft.DRIVE_MOTOR_PORT,
        Constants.FrontLeft.TURN_MOTOR_PORT, 
        Math.PI / 2,

        Constants.FrontLeft.DRIVE_MOTOR_REVERSED, 
        Constants.FrontLeft.TURN_MOTOR_REVERSED

        // Constants.FrontLeft.ABSOLUTE_ENCODER_PORT, 
        // Constants.FrontLeft.ABSOLUTE_ENCODER_OFFSET_RAD, 
        // Constants.FrontLeft.ABOSLUTE_ENCODER_REVERSED
    );
    private final SwerveModule frontRight = new SwerveModule(
        Constants.FrontRight.DRIVE_MOTOR_PORT,
        Constants.FrontRight.TURN_MOTOR_PORT, 
        Math.PI / 2,

        Constants.FrontRight.DRIVE_MOTOR_REVERSED, 
        Constants.FrontRight.TURN_MOTOR_REVERSED

        // Constants.FrontRight.ABSOLUTE_ENCODER_PORT, 
        // Constants.FrontRight.ABSOLUTE_ENCODER_OFFSET_RAD, 
        // Constants.FrontRight.ABOSLUTE_ENCODER_REVERSED
    );
    private final SwerveModule backLeft = new SwerveModule(
        Constants.BackLeft.DRIVE_MOTOR_PORT,
        Constants.BackLeft.TURN_MOTOR_PORT, 
        Math.PI / 2,

        Constants.BackLeft.DRIVE_MOTOR_REVERSED, 
        Constants.BackLeft.TURN_MOTOR_REVERSED

        // Constants.BackLeft.ABSOLUTE_ENCODER_PORT, 
        // Constants.BackLeft.ABSOLUTE_ENCODER_OFFSET_RAD, 
        // Constants.BackLeft.ABOSLUTE_ENCODER_REVERSED
    );
    private final SwerveModule backRight = new SwerveModule(
        Constants.BackRight.DRIVE_MOTOR_PORT,
        Constants.BackRight.TURN_MOTOR_PORT, 
        Math.PI / 2,
        

        Constants.BackRight.DRIVE_MOTOR_REVERSED, 
        Constants.BackRight.TURN_MOTOR_REVERSED

        // Constants.BackRight.ABSOLUTE_ENCODER_PORT, 
        // Constants.BackRight.ABSOLUTE_ENCODER_OFFSET_RAD, 
        // Constants.BackRight.ABOSLUTE_ENCODER_REVERSED
    );
    private final SwerveModule[] swerveModules = { frontLeft, frontRight, backLeft, backRight };

    // private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    private final Pigeon2 pigeon2 = new Pigeon2(15);

    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(
        Constants.DRIVE_KINEMATICS, 
        new Rotation2d(0), 
        getModulePositions()
    );

    private volatile boolean isFieldOriented = true; //TODO:CHSNGE!!

    public Drive() {
        //TODO: Make sure IMU RESETS
        for (SwerveModule swerveModule: swerveModules) {
            swerveModule.breakMode();
        }

        pigeon2.configMountPose(AxisDirection.NegativeX, AxisDirection.PositiveZ);
        // new Thread(() -> {
        //     try {
        //         Thread.sleep(1000);
        //         resetHeading();
        //     } catch (Exception ignoredException) {
        //         // TODO: add an error message or something here if possible
        //     }
        // }).start();


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

        SmartDashboard.putNumber("Front Left Turn Distance", frontLeft.getTurnEncoderRad());
        SmartDashboard.putNumber("Front Right Turn Distance", frontRight.getTurnEncoderRad());
        SmartDashboard.putNumber("Back Left Turn Distance", backLeft.getTurnEncoderRad());
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

    private double headingOffset = 0;

    public double getHeading() {
        // return Math.IEEEremainder(gyro.getAngle(), 360);
        return Math.IEEEremainder(-pigeon2.getYaw(), 360) - headingOffset;
    }
    
    public void resetHeading() {
        headingOffset += getHeading();
        // gyro.reset();
        // pigeon2.zeroGyroBiasNow();
        // pigeon2.setYaw(0);
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

    public void stop() {
        for (SwerveModule swerveModule: swerveModules) {
            swerveModule.stop();
        }
    }

    public void setModuleStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveModule.Constants.PHYSICAL_MAX_SPEED_METERS_PER_SECOND);
        for (int i = 0; i < 4; i++) {
            swerveModules[i].setState(states[i]);
        }
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        };
    }

    public boolean isFieldOriented() {
        return isFieldOriented;
    }

    public void toggleFieldOriented() {
        isFieldOriented = !isFieldOriented;
    }

    public CommandBase runToggleFieldOriented() {
        return runOnce(this::toggleFieldOriented);
    }

    public CommandBase runResetHeading() {
        return runOnce(this::resetHeading);
    }

    public void resetEncoders() {
        for (SwerveModule swerveModule: swerveModules) {
            swerveModule.resetEncoders();
        }
    }

    public CommandBase runResetEncoders() {
        return runOnce(this::resetEncoders);
    }

    private static final double POWER = 0.1;

    public CommandBase frontLeft() {
        return runOnce(() -> frontLeft.set(POWER));
    }

    public CommandBase frontRight() {
        return runOnce(() -> frontRight.set(POWER));
    }
    public CommandBase backLeft() {
        return runOnce(() -> backLeft.set(POWER));
    }
    public CommandBase backRight() {
        return runOnce(() -> backRight.set(POWER));
    }

    public CommandBase runStop() {
        return runOnce(this::stop);
    }

    
    
        
}
