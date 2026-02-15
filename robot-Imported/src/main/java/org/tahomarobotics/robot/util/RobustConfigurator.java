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

package org.tahomarobotics.robot.util;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.tinylog.Logger;

import java.util.function.Consumer;
import java.util.function.Supplier;

@SuppressWarnings("UnusedReturnValue")
public class RobustConfigurator {
    /**
     * Number of configuration attempts.
     */
    private static final int RETRIES = 5;

    // Retrying Configurator

    /**
     * Attempts to run the configuration function until success or RETRIES.
     *
     * @param specifier Specifier for the device(s)
     * @param config    Configuration function
     *
     * @return Resulting status code
     */
    @SuppressWarnings("SameParameterValue")
    private static StatusCode tryConfigure(String specifier, Supplier<StatusCode> config) {
        StatusCode statusCode = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < RETRIES; i++) {
            statusCode = config.get();
            if (statusCode.isOK()) {
                Logger.info("Successfully configured {} in {} attempt{}!", specifier, i + 1, i == 0 ? "" : "s");
                break;
            } else if (statusCode.isWarning()) {
                Logger.warn(
                    "[{}/{}] Configuring {} returned warning status code: {}, retrying...", i + 1, RETRIES, specifier,
                    statusCode
                );
            } else {
                Logger.error(
                    "[{}/{}] Configuring {} returned error status code: {}, retrying...", i + 1, RETRIES, specifier,
                    statusCode
                );
            }
        }
        return statusCode;
    }

    // Device Configurators

    /**
     * Attempts to configure a TalonFX.
     *
     * @param deviceName    Name of the device
     * @param motor         The motor
     * @param configuration Configuration to apply
     *
     * @return The resulting status code
     */
    public static StatusCode tryConfigureTalonFX(String deviceName, TalonFX motor, TalonFXConfiguration configuration) {
        return tryConfigure("TalonFX '" + deviceName + "'", () -> motor.getConfigurator().apply(configuration));
    }

    /**
     * Attempts to modify the configuration of a TalonFX.
     *
     * @param deviceName   Name of the device
     * @param motor        The motor
     * @param modification Modification to apply
     *
     * @return The resulting status code
     */
    public static StatusCode tryModifyTalonFX(
        String deviceName, TalonFX motor, Consumer<TalonFXConfiguration> modification) {
        var config = new TalonFXConfiguration();
        motor.getConfigurator().refresh(config);
        modification.accept(config);

        return tryConfigureTalonFX(deviceName, motor, config);
    }

    /**
     * Attempts to configure a CANcoder.
     *
     * @param deviceName    Name of the device
     * @param encoder       The encoder
     * @param configuration Configuration to apply
     *
     * @return The resulting status code
     */
    public static StatusCode tryConfigureCANcoder(
        String deviceName, CANcoder encoder, CANcoderConfiguration configuration) {
        return tryConfigure("CANcoder '" + deviceName + "'", () -> encoder.getConfigurator().apply(configuration));
    }

    /**
     * Attempts to modify the configuration of a CANcoder.
     *
     * @param deviceName   Name of the device
     * @param encoder      The encoder
     * @param modification Modification to apply
     *
     * @return The resulting status code
     */
    public static StatusCode tryModifyCANcoder(
        String deviceName, CANcoder encoder, Consumer<CANcoderConfiguration> modification) {
        var config = new CANcoderConfiguration();
        encoder.getConfigurator().refresh(config);
        modification.accept(config);

        return tryConfigureCANcoder(deviceName, encoder, config);
    }

    /**
     * Attempts to configure a CANrange
     * @param deviceName    Name of the device
     * @param canRange      The range sensor
     * @param configuration Configuration to apply
     * @return the resulting status code
     */
    public static StatusCode tryConfigureCANrange(
        String deviceName, CANrange canRange, CANrangeConfiguration configuration) {
        return tryConfigure("CANrange '" + deviceName + "'", () -> canRange.getConfigurator().apply(configuration));
    }

    /**
     * Attempts to modify the configuration of a CANrange
     * @param deviceName   Name of the device
     * @param canRange     The range sensor
     * @param modification Modification to apply
     * @return the resulting status code
     */
    public static StatusCode tryModifyCANrange(
        String deviceName, CANrange canRange, Consumer<CANrangeConfiguration> modification) {
        var config = new CANrangeConfiguration();
        canRange.getConfigurator().refresh(config);
        modification.accept(config);

        return tryConfigureCANrange(deviceName, canRange, config);
    }


    // Helper Methods

    /**
     * Attempts to set the angular offset of a CANcoder.
     *
     * @param deviceName Name of the device
     * @param encoder    The encoder
     * @param offset     The new offset of the CANcoder in <strong>rotations</strong>
     *
     * @return The resulting status code
     */
    public static StatusCode trySetCANcoderAngularOffset(String deviceName, CANcoder encoder, double offset) {
        return tryModifyCANcoder(deviceName, encoder, e -> e.MagnetSensor.MagnetOffset = offset);
    }

    /**
     * Attempts to set the neutral mode of a TalonFX.
     *
     * @param deviceName Name of the device
     * @param motor      The motor
     * @param mode       The new neutral mode
     *
     * @return The resulting status code
     */
    public static StatusCode trySetMotorNeutralMode(String deviceName, TalonFX motor, NeutralModeValue mode) {
        return tryModifyTalonFX(deviceName, motor, m -> m.MotorOutput.NeutralMode = mode);
    }
}