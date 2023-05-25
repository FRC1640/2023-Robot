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

public class AlignWithTagCommand extends CommandBase{

    DriveSubsystem driveSubsystem;
    Gyro gyro;
    XboxController opController;
    Limelight limelight;

    int dpadPos;
    double tagIndex;
    Pose2d robotPos;

    // should be the constant coords (x and y) of the left, right, and center pos on the  xxxxx"robot in april space"

    // Blue Side - left tag (ID 6)
    double blueX = 3; //2; // distance to pole???
    double blueLeftTagLeftConeY = 4.9; // 4.9 plus 8??
    double blueLeftTagRightConeY = 3.9; 
    double blueLeftTagMiddleY = 4.45; 

    Translation2d tag6Coords = new Translation2d(blueX, blueLeftTagMiddleY);


    //  Blue Side - middle tag (ID 7)

    double blueMiddleTagLeftConeY = 3.2; 
    double blueMiddleTagRightConeY = 2.25; 
    double blueMiddleTagMiddleY = 2.75; 

    Translation2d tag7Coords = new Translation2d(blueX, blueMiddleTagMiddleY);



    //  Blue Side - right tag (ID 8)
    double blueRightTagMiddleY = 1.1;
    double blueRightTagLeftConeY = 1.7;
    double blueRightTagRightConeY = 0.5;

    Translation2d tag8Coords = new Translation2d(blueX, blueRightTagMiddleY);

    // Red Side Tags
    Translation2d tag1Coords= new Translation2d(100, 100); 
    Translation2d tag2Coords= new Translation2d(100, 100); 
    Translation2d tag3Coords= new Translation2d(100, 100); 



    Translation2d[] tagCoordinates = {tag1Coords, tag2Coords, tag3Coords, tag6Coords, tag7Coords, tag8Coords};

    

    double targetX;
    double targetY;

    double distToTag;
    Translation2d closestTagCoords;
    int closestTagIndex;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(Math.PI, Math.PI);
    public static final double x = Units.inchesToMeters(10.375); // 10.375"
    public static final double y = Units.inchesToMeters(12.375); // 12.375"
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(new Translation2d(y, x),new Translation2d(y, -x), new Translation2d(-y, x), new Translation2d(-y, -x));
   


    public AlignWithTagCommand(DriveSubsystem driveSubsystem,  Gyro gyro, Limelight limelight ){ //XboxController opController,
        
        this.driveSubsystem = driveSubsystem;
        this.gyro = gyro;
        // this.opController = opController;
        this.limelight = limelight;

    }

   
    public void initialize() {
        
    }
    
    public void execute() {

        //dpadPos = opController.getPOV();
        tagIndex = limelight.getAprilTagIndex();
        System.out.println("TAG INDEX: " + tagIndex);

        gyro.resetGyro();
        //robotPos = limelight.getRobotInAprilTagSpace();
        robotPos = limelight.getBotPose();
        gyro.setOffset(-limelight.getBotPose().getRotation().getDegrees());
                
        driveSubsystem.resetOdometry(new Pose2d(new Translation2d(limelight.getBotPose().getX() + 8, limelight.getBotPose().getY() + 4), limelight.getBotPose().getRotation()));
        Translation2d robotCoords = robotPos.getTranslation(); // x and y from x y and rotation
        

        if (tagIndex == 6){ // tag 6 is blue left 
            targetX = blueX;
            targetY = blueLeftTagMiddleY;
        } 
        else if(tagIndex == 7){ // tag 7 is blue middle
            targetX = blueX;
            targetY = blueMiddleTagMiddleY; 
        }
        else if(tagIndex == 8){ // tag 8 is blue right
            targetX = blueX;
            targetY = blueRightTagMiddleY; 
        }
            
        
        // making the path eek
        PathPlannerTrajectory alignWithGrid = PathPlanner.generatePath(
            new PathConstraints(2, 1),
            new PathPoint(robotCoords, new Rotation2d(0), new Rotation2d(robotPos.getRotation().getRadians())),
            new PathPoint(new Translation2d(targetX, targetY), new Rotation2d(0), new Rotation2d(0))
          );

        PPSwerveControllerCommand path = new PPSwerveControllerCommand(alignWithGrid, 
        () -> driveSubsystem.getPose(), // Functional interface to feed supplier
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
