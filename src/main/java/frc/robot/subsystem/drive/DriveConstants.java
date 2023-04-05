package frc.robot.subsystem.drive;

import edu.wpi.first.math.util.Units;
import frc.robot.utility.ShuffleboardValue;
import frc.robot.utility.ShuffleboardValueEnum;

public class DriveConstants {
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

        TRANSLATIONAL_KP(1.8), //0.08361 //1.9-0.09
        TRANSLATIONAL_KI(0),
        TRANSLATIONAL_KD(0),

        THETA_KP(0.7), //0.01 //Changed to 1 from 0.2 (Lucky) =0.4 0.08
        THETA_KI(0),
        THETA_KD(0),

        FRONT_LEFT_ABSOLUTE_ENCODER_OFFSET_RADIANS(2.774966),
        FRONT_RIGHT_ABSOLUTE_ENCODER_OFFSET_RADIANS(3.854886),
        BACK_LEFT_ABSOLUTE_ENCODER_OFFSET_RADIANS(2.589354),
        BACK_RIGHT_ABSOLUTE_ENCODER_OFFSET_RADIANS(3.334867),

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
        NORMAL(2.5, 1),
        SLOW(0.2, 0.2),
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
}
