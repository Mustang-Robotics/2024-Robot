package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.BottomShooter;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.TopShooter;


public class SetArmAim extends Command {
    
    public final Arm m_arm;//
    
    public final DriveSubsystem m_drive;
    public final TopShooter m_top;
    public final BottomShooter m_bottom;

    public SetArmAim(Arm arm, DriveSubsystem drive, TopShooter top, BottomShooter bottom){
        m_arm = arm;
        m_drive = drive;
        m_top = top;
        m_bottom = bottom;
        addRequirements(m_arm, m_top, m_bottom);
    }
    
    @Override
    public void execute(){
        m_arm.setGoal(Constants.Arm.goal.get(m_drive.vision.LocationToSpeaker()[0]));
        m_top.setSetpoint(3000);
        m_bottom.setSetpoint(3000);
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted){
        m_top.setSetpoint(0);
        m_bottom.setSetpoint(0);
    }
}
