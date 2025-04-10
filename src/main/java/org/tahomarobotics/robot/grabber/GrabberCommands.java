/*
 * Copyright 2025 Tahoma Robotics
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

package org.tahomarobotics.robot.grabber;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.tahomarobotics.robot.collector.Collector;
import org.tahomarobotics.robot.util.game.GamePiece;
import org.tahomarobotics.robot.windmill.Windmill;
import org.tahomarobotics.robot.windmill.WindmillConstants;

import java.util.Set;

public class GrabberCommands {
    public static Pair<Command, Command> createGrabberCommands(Grabber grabber) {
        Command onTrue = Commands.defer(() -> (Windmill.getInstance().getTargetTrajectoryState() == WindmillConstants.TrajectoryState.L1) ?
            Commands.runOnce(grabber::transitionToPullingL1) :
            (Collector.getInstance().getCollectionMode().equals(GamePiece.CORAL)) ?
                Commands.runOnce(grabber::transitionToCoralCollecting).onlyIf(() -> !grabber.isHoldingAlgae()) : Commands.runOnce(grabber::transitionToAlgaeCollecting),
            Set.of(grabber));
        Command onFalse = grabber.runOnce(grabber::transitionToDisabled).onlyIf(() -> !grabber.isHoldingAlgae() && !grabber.algaeCollectionTimer.isRunning());

        return Pair.of(onTrue, onFalse);
    }

    public static Pair<Command, Command> createGrabberScoringCommands(Grabber grabber) {
        Command onTrue = Commands.defer(() -> (Windmill.getInstance().getTargetTrajectoryState() == WindmillConstants.TrajectoryState.L1) ?
            Commands.runOnce(grabber::transitionToScoringL1) : Commands.runOnce(grabber::transitionToScoring), Set.of(grabber));
        Command onFalse = grabber.runOnce(grabber::transitionToDisabled).onlyIf(() -> !grabber.coralCollectionTimer.isRunning());

        return Pair.of(onTrue, onFalse);
    }
}