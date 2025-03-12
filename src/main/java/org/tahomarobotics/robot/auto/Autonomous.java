package org.tahomarobotics.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.tahomarobotics.robot.auto.autos.FivePiece;
import org.tahomarobotics.robot.util.SubsystemIF;
import org.tinylog.Logger;

public class Autonomous extends SubsystemIF {
    private static final Autonomous INSTANCE = new Autonomous();

    private final SendableChooser<Command> autoChooser;

    private Autonomous() {
        autoChooser = new SendableChooser<>();
        autoChooser.setDefaultOption("No-Op", Commands.none().withName("No-Op"));

        autoChooser.addOption("5-Piece Left", Commands.deferredProxy(() -> new FivePiece(true)).withName("5-Piece Left"));
        autoChooser.addOption("5-Piece Right", Commands.deferredProxy(() -> new FivePiece(false)).withName("5-Piece Right"));

        // Force-Load the reef positions
        AutonomousConstants.getNearestReefPoleScorePosition(new Pose2d().getTranslation()).approachPose().getX();

        SmartDashboard.putData("Auto Chooser", autoChooser);

        autoChooser.onChange(command -> {
            Logger.info("Selected auto: " + command.getName());
        });
    }

    public static Autonomous getInstance() {
        return INSTANCE;
    }

    public Command getSelectedAuto() {
        return autoChooser.getSelected();
    }
}