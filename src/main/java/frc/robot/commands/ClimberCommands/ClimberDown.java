package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberDown extends Command {

    private final ClimberSubsystem climberSubsystem;

    public ClimberDown(ClimberSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;
        addRequirements(climberSubsystem);
    }

    public void execute() {
        climberSubsystem.ClimberDown();
    }
}
