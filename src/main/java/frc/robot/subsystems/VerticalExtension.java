package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DroidRageConstants;

public class VerticalExtension extends SubsystemBase {
    public static class Constants {
        public static final double GEAR_RATIO = 1 / 1;
        public static final double GEAR_DIAMETER_METERS = 0.0481; // 1.893 inches
        public static final double ROT_TO_METER = (GEAR_RATIO * Math.PI * GEAR_DIAMETER_METERS);
        

    }
    private enum Position {
        GROUND(0),
        MID(0),
        HIGH(0),
        INTAKE(0),
        ;

        private final double height_meters;

        private Position(double height_meters) {
            this.height_meters = height_meters;
        }
    }
    
    private final CANSparkMax leftElevator, rightElevator;
    private final DutyCycleEncoder encoder;
    private final PIDController controller;
    private volatile Position position;

    
    public VerticalExtension() {
        leftElevator = new CANSparkMax(0, MotorType.kBrushless);
        rightElevator = new CANSparkMax(0, MotorType.kBrushless);//TODO: Where is it plugged in?

        leftElevator.setIdleMode(IdleMode.kBrake);
        rightElevator.setIdleMode(IdleMode.kBrake);
        rightElevator.follow(leftElevator, true);

  
        encoder = new DutyCycleEncoder(0);  //TODO: Where is it plugged in?
        encoder.setDistancePerRotation(Constants.ROT_TO_METER);

        controller = new PIDController(0, 0, 0);
        controller.setTolerance(0.10); // meters

        setPosition(Position.GROUND);
    }

    @Override
    public void periodic() {
        DroidRageConstants.putNumber("Vertical Elevator/Height", encoder.get());
    }
  
    @Override
    public void simulationPeriodic() {
        periodic();
    }

    public double getTargetHeight() {
        return DroidRageConstants.getNumber("Vertical Elevator/Position/"+ position.name(), position.height_meters);
    }

    private CommandBase setPosition(Position position) {
        return runOnce(() -> {
            this.position = position;
            controller.setSetpoint(getTargetHeight());
            DroidRageConstants.putString("Vertical Elevator/Position", position.name());
        });
    }

    public CommandBase moveGround() {
        return setPosition(Position.GROUND);
    }

    public CommandBase moveMid() {
        return setPosition(Position.MID);
    }

    public CommandBase moveHigh() {
        return setPosition(Position.HIGH);
    }

    public CommandBase moveIntake() {
        return setPosition(Position.INTAKE);
    }

    public CommandBase moveToPosition() {
        return runOnce(() -> leftElevator.set(controller.calculate(encoder.get())));
    }
}  
