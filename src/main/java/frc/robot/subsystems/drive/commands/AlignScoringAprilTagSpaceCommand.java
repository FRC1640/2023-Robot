package frc.robot.subsystems.drive.commands;
import java.sql.Array;

// import all of the path planner stuff <3
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

// xbox controller for the opperator and joystick for.. the dpad?
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;


import edu.wpi.first.wpilibj2.command.CommandBase;

// limelight (to see fr)
import frc.robot.sensors.Limelight;

// driving import stuff
import frc.robot.sensors.Gyro;
import frc.robot.subsystems.drive.DriveSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d; 
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class AlignScoringAprilTagSpaceCommand extends CommandBase{

    DriveSubsystem driveSubsystem;
    Gyro gyro;
    XboxController opController;
    Limelight limelight;

    int dpadPos;
    double tagIndex;
    Pose2d robotPos;

    // should be the constant coords (x and y) of the left, right, and center pos on the "robot in april space"
    double targetY= 0.8; 

    double sideConeX = 0.5;   
    double middleX = 0;
    double targetX;
   
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(Math.PI, Math.PI);
    public static final double x = Units.inchesToMeters(10.375); // 10.375"
    public static final double y = Units.inchesToMeters(12.375); // 12.375"
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(new Translation2d(y, x),new Translation2d(y, -x), new Translation2d(-y, x), new Translation2d(-y, -x));
   


    public AlignScoringAprilTagSpaceCommand(DriveSubsystem driveSubsystem, XboxController opController, Gyro gyro, Limelight limelight ){
        
        this.driveSubsystem = driveSubsystem;
        this.gyro = gyro;
        this.opController = opController;
        this.limelight = limelight;

    }

   
    public void initialize() {
        
    }
    
    public void execute() {

        dpadPos = opController.getPOV();
        //tagIndex = limelight.getAprilTagIndex();

        gyro.resetGyro();
        //robotPos = limelight.getRobotInAprilTagSpace();
        robotPos = limelight.getRobotInAprilTagSpace();
        
        driveSubsystem.resetOdometry(new Pose2d(new Translation2d(limelight.getBotPose().getX() + 8, limelight.getBotPose().getY() + 4), limelight.getBotPose().getRotation()));
        Translation2d robotCoords = robotPos.getTranslation(); // x and y from x y and rotation
    


        if (dpadPos == 270){ // IF LEFT ON THE DPAD      
            targetX = robotCoords.getX() - sideConeX; 
        }
        else if (dpadPos == 270){ // IF RIGHT ON THE DPAD
            targetX = Math.abs(robotCoords.getX()) - sideConeX;
        }
        else if(dpadPos == 0){ //IF UP ON THE DPAD
            targetX = middleX; 
        }

        PathPlannerTrajectory alignWithGrid = PathPlanner.generatePath(
            new PathConstraints(2, 1),
            new PathPoint(robotCoords, new Rotation2d(0), new Rotation2d(robotPos.getRotation().getRadians())),
            new PathPoint(new Translation2d(targetX, targetY), new Rotation2d(0), new Rotation2d(0))
          );


        PPSwerveControllerCommand path = new PPSwerveControllerCommand(alignWithGrid, 
        ()-> robotPos, // Functional interface to feed supplier
        kDriveKinematics, new PIDController(0.4, 0.0, 0.0), new PIDController(0.4, 0.0, 0.0), new PIDController(0.5, 0, 0),
        driveSubsystem::setModuleStates, false, driveSubsystem);

        path.schedule();

    }
        


    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }









    
}
