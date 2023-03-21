package frc.robot.subsystem;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

    // Visit Limelight Web interface at http://10.30.35.11:5801
    
    // Initialize Limelight network tables
    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private NetworkTableEntry tx = table.getEntry("tx");
    private NetworkTableEntry ty = table.getEntry("ty");
    private NetworkTableEntry ta = table.getEntry("ta");
    private NetworkTableEntry tv = table.getEntry("tv");

    // Read initial values
    private double x = tx.getDouble(0.0);
    private double y = ty.getDouble(0.0);
    private double area = ta.getDouble(0.0);
    private double v = tv.getDouble(0.0);

    // public static enum LightMode {
	// 	eOn, eOff, eBlink
	// }

    public Vision() {
        super();
    }

    @Override
    public void periodic() {
        // Read values periodically
        x = tx.getDouble(0.0);
        y = ty.getDouble(0.0);
        area = ta.getDouble(0.0);

        // Post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);
        SmartDashboard.putNumber("LimelightV", v);

    }

    @Override
    public void simulationPeriodic() {
        periodic();
    }
}
