package frc.robot.utils;

import java.util.HashMap;
import java.util.Map;

import com.google.common.collect.ImmutableMap;

import choreo.auto.AutoChooser;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.Swerve;

public class AutoBuilder {

    private final Swerve drive;
    private final CommandXboxController controller;
    private final StateMachine stateMachine;

    public AutoBuilder (Swerve drive, CommandXboxController controller, StateMachine stateMachine) {
        this.drive = drive;
        this.controller = controller;
        this.stateMachine = stateMachine;
    }

    public SendableChooser<SequentialCommandGroup> build () {
        var map = ImmutableMap.<String, CommandBuilder>builder()
            .put("test1",
                new CommandBuilder(drive, controller, stateMachine)
            )
            .put("test2",
                new CommandBuilder(drive, controller, stateMachine)
            )
            .build()
        ;

        var chooser = new SendableChooser<SequentialCommandGroup>();
        var keys = map.keySet().toArray();
        for (int i = 0; i < keys.length; i++) {
            if (i == 0) {
                chooser.setDefaultOption((String) keys[0], map.get(keys[0]).build());
                continue;
            }

            chooser.addOption((String) keys[i], map.get(keys[i]).build());
        }

        return chooser;
    }
}
