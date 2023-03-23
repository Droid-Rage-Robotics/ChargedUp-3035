package frc.robot.subsystem.arm.pivot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.utility.ShuffleboardValue;

public class PivotMotionProfiled extends Pivot {
    public static class Constants {
        public static double MIN_POSITION = Math.toRadians(20);
        public static double MAX_POSITION = Math.toRadians(200);
    }
    protected TrapezoidProfile profile;
    protected final TrapezoidProfile.Constraints constraints;
    protected TrapezoidProfile.State state;
    protected TrapezoidProfile.State goal;

    protected final ShuffleboardValue<Double> goalPositionWriter = ShuffleboardValue.create(0.0, "Goal Position", Pivot.class.getSimpleName()).build();
    protected final ShuffleboardValue<Double> goalVelocityWriter = ShuffleboardValue.create(0.0, "Goal Velocity", Pivot.class.getSimpleName()).build();
    protected final ShuffleboardValue<Double> targetVelocityWriter = ShuffleboardValue.create(0.0, "Target Velocity", Pivot.class.getSimpleName()).build();
    
    public PivotMotionProfiled() {
        super();

        controller.setPID(5, 0, 0);
        controller.setTolerance(0.10);

        feedforward = new ArmFeedforward(0.079284, 0.12603, 2.3793, 0.052763);
        // feedforward = new ArmFeedforward(0,0,0,0);

        constraints = new TrapezoidProfile.Constraints(
            4, // radians per second // The theorical max velocity 0.104719755 rad/s
            8 //radians per second per second
        );

        state = new TrapezoidProfile.State(0, 0);
        goal = new TrapezoidProfile.State(0, 0);
        
        profile = new TrapezoidProfile(constraints, goal, state);
    }

    @Override
    public void periodic() {
        if (isMovingManually()) {
            double encoderPosition = getEncoderPosition();
            if (encoderPosition < Constants.MIN_POSITION) {
                stop();
                return;
            }
            if (encoderPosition > Constants.MAX_POSITION) {
                stop();
                return;
            }
            setVoltage(calculateFeedforward(encoderPosition, getTargetVelocity()));
            return;
        } 
        profile = new TrapezoidProfile(constraints, goal, state);
        state = profile.calculate(0.02); // 0.02 taken from TrapezoidProfileSubsystem measured in seconds
        setVoltage(calculateFeedforward(state.position, state.velocity) + calculatePID(state.position));
        getEncoderVelocity();
        
    }
  
    @Override
    public void simulationPeriodic() {
        periodic();
    }

    @Override
    public void setTargetPosition(double positionRadians) {
        setMovingManually(false);
        setTarget(positionRadians, 0);
    }

    public void setTargetVelocity(double velocityRadiansPerSecond) {
        setMovingManually(true);
        targetVelocityWriter.write(velocityRadiansPerSecond);
    }

    public double getTargetVelocity() {
        return targetVelocityWriter.get();
    }

    public void setTarget(double positionRadians, double velocityRadiansPerSecond) {
        setMovingManually(false);
        if (positionRadians < Constants.MIN_POSITION) return;
        if (positionRadians > Constants.MAX_POSITION) return;
        goalPositionWriter.write(positionRadians);
        goalVelocityWriter.write(velocityRadiansPerSecond);
        state = new TrapezoidProfile.State(getEncoderPosition(), getEncoderVelocity());
        goal = new TrapezoidProfile.State(positionRadians, velocityRadiansPerSecond);
    }
}  