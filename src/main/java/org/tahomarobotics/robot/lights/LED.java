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

package org.tahomarobotics.robot.lights;

import com.ctre.phoenix.CANifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.util.Color;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.collector.Collector;
import org.tahomarobotics.robot.util.SubsystemIF;

/*
LED Channels
 - A = green
 - B = red
 - C = blue
 */
public class LED extends SubsystemIF {
    private static final LED INSTANCE = new LED();

    private CANifier canifier = null;

    private LED() {
        if (RobotBase.isSimulation()) {
            return;
        }

        canifier = new CANifier(RobotMap.LED);
    }

    @Override
    public SubsystemIF initialize() {
        setColor(Color.kGold);
        return this;
    }

    public static LED getInstance() {
        return INSTANCE;
    }

    public void updateLEDColor() {
        if (RobotState.isEnabled()) {
            switch (Collector.getInstance().getCollectionMode()) {
                case ALGAE -> setColor(new Color(0, 1, 0.25));
                case CORAL -> setColor(Color.kWhite);
                default -> setColor(Color.kGold);
            }
        } else {
            setColor(Color.kGold);
        }
    }

    public void setColor(Color color) {
        if (canifier == null) { return; }

        canifier.setLEDOutput(color.red, CANifier.LEDChannel.LEDChannelB);
        canifier.setLEDOutput(color.green, CANifier.LEDChannel.LEDChannelA);
        canifier.setLEDOutput(color.blue, CANifier.LEDChannel.LEDChannelC);
    }

    @Override
    public void periodic() {
        updateLEDColor();
    }
}