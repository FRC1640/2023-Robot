package frc.robot.subsystems.arm.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmKinematics;
import frc.robot.subsystems.arm.ArmMath;
import frc.robot.subsystems.arm.ArmSubsystem;

public class ArmEndEffectorCommand extends CommandBase{
    ArmSubsystem armSubsystem;
    XboxController controller;
    ArmMath math = new ArmMath(Units.inchesToMeters(39), Units.inchesToMeters(35.4));
    ArmKinematics kinematics = new ArmKinematics(Units.inchesToMeters(39), Units.inchesToMeters(35.4));
    public ArmEndEffectorCommand(ArmSubsystem armSubsystem, XboxController controller) {
        this.armSubsystem = armSubsystem;
        this.controller = controller;
        addRequirements(armSubsystem);
    }
    
    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        math.setVx(MathUtil.applyDeadband(controller.getRightX(), 0.15) * 1);
        math.setVy(MathUtil.applyDeadband(controller.getRightY(), 0.15) * 1);
        math.setTheta1(Math.toRadians(armSubsystem.getLowerPosition()));
        math.setTheta2(Math.toRadians(armSubsystem.getUpperPosition()));
        
        math.inverseKinematics();
        // kinematics.setLowerAngle(Math.toRadians(armSubsystem.getLowerPosition()));
        // kinematics.setUpperAngle(Math.toRadians(armSubsystem.getUpperPosition()));
        // kinematics.fowardKinematics();
        double uVoltage = math.getOmega2() * 12;
        double lVoltage = math.getOmega1() * 12;
        // if (Units.metersToInches(kinematics.getX()) >= 47){
        //     uVoltage = Math.min(0, uVoltage);
        //     lVoltage = Math.min(0, lVoltage);
        // }
        armSubsystem.setLowerVoltage(lVoltage);
        armSubsystem.setUpperVoltage(uVoltage);
        
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
