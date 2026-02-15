package frc.robot.utils;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.Swerve;

public class CommandBuilder {
    private final List<Command> commands = new ArrayList<>();

    private final Swerve drive;
    private final CommandXboxController controller;
    private final StateMachine stateMachine;

    public CommandBuilder (Swerve drive, CommandXboxController controller, StateMachine stateMachine) {
        this.drive = drive;
        this.controller = controller;
        this.stateMachine = stateMachine;

    }

    public CommandBuilder runOnce (Runnable action) {
        this.commands.add(Commands.runOnce(action));

        return this;
    }

    public CommandBuilder wait (double secs) {
        this.commands.add(new WaitCommand(secs));


        return this;
    }

    public CommandBuilder drive (boolean autoDrive, Pose2d pose) {
        this.commands.add(new SwerveDriveCommand(this.drive, this.controller, this.stateMachine, autoDrive, pose));

        return this;
    }

    public CommandBuilder drive (Pose2d pose) {
        this.commands.add(new SwerveDriveCommand(this.drive, this.controller, this.stateMachine, true, pose));

        return this;
    }

    public CommandBuilder command (Command command) {
        this.commands.add(command);

        return this;
    }


    public SequentialCommandGroup build () {
        return new SequentialCommandGroup(this.commands.toArray(new Command[0]));
    }
}
