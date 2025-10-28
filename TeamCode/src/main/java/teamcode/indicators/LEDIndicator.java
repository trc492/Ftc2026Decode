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

package teamcode.indicators;

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
    public static final String UNKNOWN_BLOB = "Unknown";
    public static final String RED_APRILTAG = "RedAprilTag";
    public static final String BLUE_APRILTAG = "BlueAprilTag";
    public static final String NOT_FOUND = "NotFound";
    public static final String SEARCHING_RED_APRILTAG = "SearchingRedAprilTag";
    public static final String SEARCHING_BLUE_APRILTAG = "SearchingBlueAprilTag";
    public static final String DRIVE_FIELD_MODE = "FieldMode";
    public static final String DRIVE_ROBOT_MODE = "RobotMode";
    public static final String DRIVE_INVERTED_MODE = "InvertedMode";
    public static final String OFF_PATTERN = "Off";

    public final TrcPriorityIndicator.Pattern[] statusLEDPatternPriorities = new TrcPriorityIndicator.Pattern[]
    {
        // Highest priority.
        new TrcPriorityIndicator.Pattern(PURPLE_BLOB, TrcRevBlinkin.RevLedPattern.SolidViolet),
        new TrcPriorityIndicator.Pattern(GREEN_BLOB, TrcRevBlinkin.RevLedPattern.SolidGreen),
        new TrcPriorityIndicator.Pattern(UNKNOWN_BLOB, TrcRevBlinkin.RevLedPattern.SolidYellow),
        new TrcPriorityIndicator.Pattern(RED_APRILTAG, TrcRevBlinkin.RevLedPattern.SolidRed),
        new TrcPriorityIndicator.Pattern(BLUE_APRILTAG, TrcRevBlinkin.RevLedPattern.SolidBlue),
        new TrcPriorityIndicator.Pattern(NOT_FOUND, TrcRevBlinkin.RevLedPattern.SolidYellow, 0.5, 0.0),
        new TrcPriorityIndicator.Pattern(SEARCHING_RED_APRILTAG, TrcRevBlinkin.RevLedPattern.SolidRed, 0.5, 0.5),
        new TrcPriorityIndicator.Pattern(SEARCHING_BLUE_APRILTAG, TrcRevBlinkin.RevLedPattern.SolidBlue, 0.5, 0.5),
        new TrcPriorityIndicator.Pattern(DRIVE_FIELD_MODE, TrcRevBlinkin.RevLedPattern.SolidAqua, 0.5, 0.0),
        new TrcPriorityIndicator.Pattern(DRIVE_ROBOT_MODE, TrcRevBlinkin.RevLedPattern.SolidWhite, 0.5, 0.0),
        new TrcPriorityIndicator.Pattern(DRIVE_INVERTED_MODE, TrcRevBlinkin.RevLedPattern.SolidOrange, 0.5, 0.0),
        new TrcPriorityIndicator.Pattern(OFF_PATTERN, TrcRevBlinkin.RevLedPattern.SolidBlack)
        // Lowest priority.
    };

    public final TrcPriorityIndicator.Pattern[] spindexerLEDPatternPriorities = new TrcPriorityIndicator.Pattern[]
    {
        // Highest priority.
        new TrcPriorityIndicator.Pattern(PURPLE_BLOB, TrcGobildaIndicatorLight.GobildaLedPattern.Violet),
        new TrcPriorityIndicator.Pattern(GREEN_BLOB, TrcGobildaIndicatorLight.GobildaLedPattern.Green),
        new TrcPriorityIndicator.Pattern(UNKNOWN_BLOB, TrcGobildaIndicatorLight.GobildaLedPattern.Yellow),
        new TrcPriorityIndicator.Pattern(OFF_PATTERN, TrcGobildaIndicatorLight.GobildaLedPattern.Black)
        // Lowest priority.
    };

    public final TrcDbgTrace tracer;
    private TrcPriorityIndicator statusIndicator = null;
    private final TrcPriorityIndicator[] spindexerIndicators = new TrcPriorityIndicator[3];

    /**
     * Constructor: Create an instance of the object.
     *
     * @param indicatorNames specifies an array of indicator hardware names, one for each LED device.
     */
    public LEDIndicator(String[] indicatorNames)
    {
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
                        statusIndicator.setPatternPriorities(statusLEDPatternPriorities);
                        statusIndicator.reset();
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
                        spindexerIndicators[0].setPatternPriorities(spindexerLEDPatternPriorities);
                        spindexerIndicators[0].reset();
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
                        spindexerIndicators[1].setPatternPriorities(spindexerLEDPatternPriorities);
                        spindexerIndicators[1].reset();
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
                        spindexerIndicators[2].setPatternPriorities(spindexerLEDPatternPriorities);
                        spindexerIndicators[2].reset();
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
                    statusIndicator.setPatternState(DRIVE_INVERTED_MODE, true);
                    statusIndicator.setPatternState(DRIVE_ROBOT_MODE, false);
                    statusIndicator.setPatternState(DRIVE_FIELD_MODE, false);
                    break;

                case ROBOT:
                    statusIndicator.setPatternState(DRIVE_INVERTED_MODE, false);
                    statusIndicator.setPatternState(DRIVE_ROBOT_MODE, true);
                    statusIndicator.setPatternState(DRIVE_FIELD_MODE, false);
                    break;

                case FIELD:
                    statusIndicator.setPatternState(DRIVE_INVERTED_MODE, false);
                    statusIndicator.setPatternState(DRIVE_ROBOT_MODE, false);
                    statusIndicator.setPatternState(DRIVE_FIELD_MODE, true);
                    break;
            }
        }
    }   //setDriveOrientation

    /**
     * This method sets the statusLED pattern ON or OFF.
     *
     * @param patternName specifies the name of the LED pattern to turn on.
     */
    public void setStatusPattern(String patternName, boolean on)
    {
        if (statusIndicator != null)
        {
            statusIndicator.setPatternState(patternName, on);
        }
    }   //setStatusPattern

    /**
     * This method clears all vision detected states.
     */
    public void setStatusVisionPatternsOff()
    {
        if (statusIndicator != null)
        {
            statusIndicator.setPatternState(PURPLE_BLOB, false);
            statusIndicator.setPatternState(GREEN_BLOB, false);
            statusIndicator.setPatternState(UNKNOWN_BLOB, false);
            statusIndicator.setPatternState(RED_APRILTAG, false);
            statusIndicator.setPatternState(BLUE_APRILTAG, false);
            statusIndicator.setPatternState(NOT_FOUND, false);
        }
    }   //setStatusVisionPatternsOff

    /**
     * This method sets the spindexerLED pattern ON for the specified slot.
     *
     * @param slot specifies the Spindexer slot.
     * @param patternName specifies the name of the LED pattern to turn on.
     */
    public void setSpindexerPatternOn(int slot, String patternName)
    {
        if (spindexerIndicators[slot] != null)
        {
            spindexerIndicators[slot].setPatternState(patternName, true);
        }
    }   //setSpindexerPatternOn

    /**
     * This method turns off the spindexeerLED for the specified slot.
     *
     * @param slot specifies the Spindexer slot.
     */
    public void setSpindexerPatternOff(int slot)
    {
        if (spindexerIndicators[slot] != null)
        {
            spindexerIndicators[slot].setPatternState(spindexerIndicators[slot].getPattern(), false);
        }
    }   //setSpindexerPatternOff

}   //class LEDIndicator
