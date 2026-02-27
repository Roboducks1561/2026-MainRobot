package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.intakeMechanism.Arm;
import frc.robot.subsystems.swerve.SwerveDrive;

public class ArmSlow extends Command {

    private final Arm arm;
    private double initTime = -1;
    private double initialPosition = 0;
    private final double timeToRun;
    private final double maxTarget;

    private boolean isFinished = false;

    public ArmSlow(Arm arm, double timeToRun, double maxTarget) {
        this.arm = arm;
        this.maxTarget = maxTarget;
        this.timeToRun = timeToRun;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        initTime = Utils.getCurrentTimeSeconds();
        initialPosition = arm.getPosition();
        isFinished = false;
    }

    @Override
    public void execute() {
        double timedPos = initialPosition + ((Utils.getCurrentTimeSeconds() - initTime)/timeToRun) * (maxTarget - initialPosition);
        if (Utils.getCurrentTimeSeconds() - initTime > timeToRun){
            arm.setPosition(timedPos);
            return;
        }
        arm.setPosition(timedPos);
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}