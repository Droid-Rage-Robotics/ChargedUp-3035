package frc.robot.commands.Drive;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive.Drive;

public class PathPlannerFollow {
    private final Drive drive;
    private final String pathName;
    private double maxVelocity = 0.3;
    private double acceleration = 0.5;
    private HashMap<String, Command> eventMap = new HashMap<>();

    private PathPlannerFollow(Drive drive, String pathName, double maxVelocity, double acceleration, HashMap<String, Command> eventMap) {
        this.drive = drive;
        this.pathName = pathName;
        this.maxVelocity = maxVelocity;
        this.acceleration = acceleration;
        this.eventMap = eventMap;
    }

    private PathPlannerFollow(Drive drive, String pathName) {
        this.drive = drive;
        this.pathName = pathName;
    }

    public static PathPlannerFollow create(Drive drive, String pathName) {
        return new PathPlannerFollow(drive, pathName);
    }

    public PathPlannerFollow setMaxVelocity(double maxVelocity) {
        return new PathPlannerFollow(drive, pathName, maxVelocity, acceleration, eventMap);
    }

    public PathPlannerFollow setAcceleration(double acceleration) {
        return new PathPlannerFollow(drive, pathName, maxVelocity, acceleration, eventMap);
    }

    public PathPlannerFollow addMarker(String name, Command toRun) {
        eventMap.put(name, toRun);
        return new PathPlannerFollow(drive, pathName, maxVelocity, acceleration, eventMap);
    }

    public CommandBase build() {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(
            pathName, 
            new PathConstraints(maxVelocity, acceleration)
        );
        

        // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
            drive::getPose, // Pose2d supplier
            drive::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
            Drive.DRIVE_KINEMATICS, // SwerveDriveKinematics
            new PIDConstants(Drive.Config.TRANSLATIONAL_KP.get(), 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
            new PIDConstants(Drive.Config.THETA_KP.get(), 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
            drive::setFeedforwardModuleStates, // Module states consumer used to output to the drive subsystem
            eventMap,
            false, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            drive // The drive subsystem. Used to properly set the requirements of path following commands
        );
        
        //TODO check if there are any other better constructors ^^

        return autoBuilder.fullAuto(pathGroup);
    }
}
