package frc.robot.commands.ArmCommands.TestingArm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ArmDown extends Command {
    
    private final ArmSubsystem arm;

    public ArmDown(ArmSubsystem armPassedIn) {
        arm = armPassedIn;
        addRequirements(armPassedIn);
    }

    public void execute() {
        arm.ArmDown();
    }
}