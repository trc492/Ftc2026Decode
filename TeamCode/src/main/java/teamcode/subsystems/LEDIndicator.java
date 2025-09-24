/*
 * Copyright (c) 2025 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package teamcode.subsystems;

import ftclib.driverio.FtcGobildaIndicatorLight;
import ftclib.driverio.FtcRevBlinkin;
import trclib.drivebase.TrcDriveBase;
import trclib.driverio.TrcGobildaIndicatorLight;
import trclib.driverio.TrcPriorityIndicator;
import trclib.driverio.TrcRevBlinkin;
import trclib.robotcore.TrcDbgTrace;

/**
 * This class encapsulates the LED controller to provide a priority indicator showing the status of the robot.
 */
public class LEDIndicator
{
    private static final String moduleName = LEDIndicator.class.getSimpleName();
    // LED device names.
    public static final String STATUS_LED_NAME = "StatusLED";
    public static final String SPINDEXER1_LED_NAME = "Spindexer1LED";
    public static final String SPINDEXER2_LED_NAME = "Spindexer2LED";
    public static final String SPINDEXER3_LED_NAME = "Spindexer3LED";

    // LED pattern names.
    public static final String PURPLE_BLOB = "Purple";
    public static final String GREEN_BLOB = "Green";
    public static final String NOT_FOUND = "NotFound";
    public static final String APRIL_TAG = "AprilTag";
    public static final String DRIVE_ORIENTATION_FIELD = "FieldMode";
    public static final String DRIVE_ORIENTATION_ROBOT = "RobotMode";
    public static final String DRIVE_ORIENTATION_INVERTED = "InvertedMode";
    public static final String OFF_PATTERN = "Off";

    public final TrcDbgTrace tracer;
    private TrcPriorityIndicator<?> statusIndicator = null;
    private final TrcPriorityIndicator<?>[] spindexerIndicators = new TrcPriorityIndicator[3];

    /**
     * Constructor: Create an instance of the object.
     *
     * @param indicatorNames specifies an array of indicator hardware names, one for each LED device.
     */
    public LEDIndicator(String[] indicatorNames)
    {
        final TrcRevBlinkin.Pattern[] statusLEDPatternPriorities = {
            // Highest priority.
            new TrcRevBlinkin.Pattern(PURPLE_BLOB, TrcRevBlinkin.RevLedPattern.SolidViolet),
            new TrcRevBlinkin.Pattern(GREEN_BLOB, TrcRevBlinkin.RevLedPattern.SolidGreen),
            new TrcRevBlinkin.Pattern(NOT_FOUND, TrcRevBlinkin.RevLedPattern.SolidRed),
            new TrcRevBlinkin.Pattern(APRIL_TAG, TrcRevBlinkin.RevLedPattern.SolidAqua),
            new TrcRevBlinkin.Pattern(DRIVE_ORIENTATION_FIELD, TrcRevBlinkin.RevLedPattern.SolidBlue),
            new TrcRevBlinkin.Pattern(DRIVE_ORIENTATION_ROBOT, TrcRevBlinkin.RevLedPattern.SolidWhite),
            new TrcRevBlinkin.Pattern(DRIVE_ORIENTATION_INVERTED, TrcRevBlinkin.RevLedPattern.SolidOrange),
            new TrcRevBlinkin.Pattern(OFF_PATTERN, TrcRevBlinkin.RevLedPattern.SolidBlack)
            // Lowest priority.
        };
        final TrcGobildaIndicatorLight.Pattern[] spindexerLEDPatternPriorities = {
            // Highest priority.
            new TrcGobildaIndicatorLight.Pattern(PURPLE_BLOB, TrcGobildaIndicatorLight.Color.Violet),
            new TrcGobildaIndicatorLight.Pattern(GREEN_BLOB, TrcGobildaIndicatorLight.Color.Green),
            new TrcGobildaIndicatorLight.Pattern(OFF_PATTERN, TrcGobildaIndicatorLight.Color.Black)
            // Lowest priority.
        };

        tracer = new TrcDbgTrace();
        for (String indicatorName: indicatorNames)
        {
            switch (indicatorName)
            {
                case STATUS_LED_NAME:
                    if (statusIndicator != null)
                    {
                        throw new IllegalArgumentException("statusIndicator already exists.");
                    }
                    else
                    {
                        tracer.traceInfo(moduleName, "Creating " + indicatorName);
                        statusIndicator = new FtcRevBlinkin(indicatorName);
                        ((FtcRevBlinkin) statusIndicator).setPatternPriorities(statusLEDPatternPriorities);
                        statusIndicator.setPatternState(OFF_PATTERN, true);
                    }
                    break;

                case SPINDEXER1_LED_NAME:
                    if (spindexerIndicators[0] != null)
                    {
                        throw new IllegalArgumentException("spindexer1Indicator already exists.");
                    }
                    else
                    {
                        tracer.traceInfo(moduleName, "Creating " + indicatorName);
                        spindexerIndicators[0] = new FtcGobildaIndicatorLight(indicatorName);
                        ((FtcGobildaIndicatorLight) spindexerIndicators[0]).setPatternPriorities(
                            spindexerLEDPatternPriorities);
                        spindexerIndicators[0].setPatternState(OFF_PATTERN, true);
                    }
                    break;

                case SPINDEXER2_LED_NAME:
                    if (spindexerIndicators[1] != null)
                    {
                        throw new IllegalArgumentException("spindexer2Indicator already exists.");
                    }
                    else
                    {
                        tracer.traceInfo(moduleName, "Creating " + indicatorName);
                        spindexerIndicators[1] = new FtcGobildaIndicatorLight(indicatorName);
                        ((FtcGobildaIndicatorLight) spindexerIndicators[1]).setPatternPriorities(
                            spindexerLEDPatternPriorities);
                        spindexerIndicators[1].setPatternState(OFF_PATTERN, true);
                    }
                    break;

                case SPINDEXER3_LED_NAME:
                    if (spindexerIndicators[2] != null)
                    {
                        throw new IllegalArgumentException("spindexer3Indicator already exists.");
                    }
                    else
                    {
                        tracer.traceInfo(moduleName, "Creating " + indicatorName);
                        spindexerIndicators[2] = new FtcGobildaIndicatorLight(indicatorName);
                        ((FtcGobildaIndicatorLight) spindexerIndicators[2]).setPatternPriorities(
                            spindexerLEDPatternPriorities);
                        spindexerIndicators[2].setPatternState(OFF_PATTERN, true);
                    }
                    break;
            }
        }
    }   //LEDIndicator

    /**
     * This method sets the statusLED to indicate the drive orientation mode of the robot.
     *
     * @param orientation specifies the drive orientation mode.
     */
    public void setDriveOrientation(TrcDriveBase.DriveOrientation orientation)
    {
        if (statusIndicator != null)
        {
            switch (orientation)
            {
                case INVERTED:
                    statusIndicator.setPatternState(DRIVE_ORIENTATION_INVERTED, true);
                    statusIndicator.setPatternState(DRIVE_ORIENTATION_ROBOT, false);
                    statusIndicator.setPatternState(DRIVE_ORIENTATION_FIELD, false);
                    break;

                case ROBOT:
                    statusIndicator.setPatternState(DRIVE_ORIENTATION_INVERTED, false);
                    statusIndicator.setPatternState(DRIVE_ORIENTATION_ROBOT, true);
                    statusIndicator.setPatternState(DRIVE_ORIENTATION_FIELD, false);
                    break;

                case FIELD:
                    statusIndicator.setPatternState(DRIVE_ORIENTATION_INVERTED, false);
                    statusIndicator.setPatternState(DRIVE_ORIENTATION_ROBOT, false);
                    statusIndicator.setPatternState(DRIVE_ORIENTATION_FIELD, true);
                    break;
            }
        }
    }   //setDriveOrientation

    /**
     * This method sets the statusLED pattern ON for a period of time and turns off automatically afterwards.
     *
     * @param patternName specifies the name of the LED pattern to turn on.
     */
    public void setStatusPattern(String patternName)
    {
        if (statusIndicator != null)
        {
            statusIndicator.setPatternState(patternName, true, 0.5);
        }
    }   //setStatusPattern

    /**
     * This method sets the spindexerLED pattern ON for a period of time and turns off automatically afterwards.
     *
     * @param slot specifies the Spindexer slot.
     * @param patternName specifies the name of the LED pattern to turn on.
     */
    public void setSpindexerPattern(int slot, String patternName)
    {
        if (spindexerIndicators[slot] != null)
        {
            spindexerIndicators[slot].setPatternState(patternName, true);
        }
    }   //setSpindexerPattern

}   //class LEDIndicator
