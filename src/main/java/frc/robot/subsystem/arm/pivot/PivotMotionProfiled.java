package frc.robot.subsystem.arm.pivot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.utility.ShuffleboardValue;

public class PivotMotionProfiled extends Pivot {
    protected TrapezoidProfile profile;
    protected final TrapezoidProfile.Constraints constraints;
    protected TrapezoidProfile.State state;
    protected TrapezoidProfile.State goal;

    protected final ShuffleboardValue<Double> goalPositionWriter = ShuffleboardValue.create(0.0, "Goal Position", Pivot.class.getSimpleName()).build();
    protected final ShuffleboardValue<Double> goalVelocityWriter = ShuffleboardValue.create(0.0, "Goal Velcotiy", Pivot.class.getSimpleName()).build();
    
    public PivotMotionProfiled() {
        super();

        controller.setPID(5, 0, 0);
        controller.setTolerance(0.10);

        feedforward = new ArmFeedforward(0.079284, 0.12603, 2.3793, 0.052763);
        // feedforward = new ArmFeedforward(0,0,0,0);

        constraints = new TrapezoidProfile.Constraints(
            2, // radians per second // The theorical max velocity 0.104719755 rad/s
            2 //radians per second per second
        );

        state = new TrapezoidProfile.State(0, 0);
        goal = new TrapezoidProfile.State(0, 0);
        
        profile = new TrapezoidProfile(constraints, goal, state);
    }

    @Override
    public void periodic() {
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
        setTarget(positionRadians, 0);
    }

    public void setTargetVelocity(double velocityRadiansPerSecond) {
        setTarget(getEncoderPosition() + velocityRadiansPerSecond, velocityRadiansPerSecond);
    }

    public void setTarget(double positionRadians, double velocityRadiansPerSecond) {
        goalPositionWriter.write(positionRadians);
        goalVelocityWriter.write(velocityRadiansPerSecond);
        state = new TrapezoidProfile.State(getEncoderPosition(), getEncoderVelocity());
        goal = new TrapezoidProfile.State(positionRadians, velocityRadiansPerSecond);
    }
}  