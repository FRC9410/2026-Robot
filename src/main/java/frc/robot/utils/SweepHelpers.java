package frc.robot.utils;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.StateMachine.GameZone;

public class SweepHelpers {
  
  public static SweepDirection convertButtonToSweep (ControllerButton key, boolean blueAlliance) {
    // hehe spagetti

    if (blueAlliance) {
      switch (key) {
        case X: return SweepDirection.TOP;
        case B: return SweepDirection.BOTTOM;
        case A: return SweepDirection.LEFT;
        default: return SweepDirection.RIGHT;
      }
    } else {
      switch (key) {
        case B: return SweepDirection.TOP;
        case X: return SweepDirection.BOTTOM;
        case Y: return SweepDirection.LEFT;
        default: return SweepDirection.RIGHT;
      }
    }
  }

  public static Optional<SequentialCommandGroup> sweep (SweepDirection direction, GameZone zone) {
    if (zone != GameZone.NEUTRAL) {
      return Optional.empty();
    }

    // TODO: implement the sweeping movements
    // it should move to the closest point of the sweep,
    // and move towards the other point until
    // the driver lets go of the button
    //
    // that may not all go here, due to the nature
    // of the interruptable sweep

    return Optional.empty();
  }

  // based off when you look at the field from top-down with blue on left
  public enum SweepDirection {
    TOP,
    BOTTOM,
    LEFT,
    RIGHT
  }

  public enum ControllerButton {
    A,
    X,
    Y,
    B
  }
}
