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

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;

import ftclib.driverio.FtcDashboard;
import ftclib.motor.FtcMotorActuator.MotorType;
import ftclib.robotcore.FtcOpMode;
import ftclib.sensor.FtcSensorTrigger;
import ftclib.subsystem.FtcPidStorage;
import teamcode.Robot;
import teamcode.RobotParams;
import teamcode.indicators.LEDIndicator;
import teamcode.vision.Vision;
import trclib.dataprocessor.TrcDataBuffer;
import trclib.dataprocessor.TrcWarpSpace;
import trclib.motor.TrcMotor;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcStateMachine;
import trclib.robotcore.TrcTaskMgr;
import trclib.sensor.TrcTrigger;
import trclib.sensor.TrcTriggerThresholdRange;
import trclib.subsystem.TrcPidStorage;
import trclib.subsystem.TrcSubsystem;

/**
 * This class implements a Spindexer Subsystem. This implementation consists of one motor with encoder, a lower limit
 * switch and entry and exit sensors.
 */
public class Spindexer extends TrcSubsystem
{
    public static final class Params
    {
        public static final String SUBSYSTEM_NAME               = "Spindexer";
        public static final boolean NEED_ZERO_CAL               = true;
        public static final boolean SPIN_INTAKE_ON_MOVE         = true;

        public static final boolean HAS_ENTRY_SENSOR            = true;
        public static final boolean DUAL_ENTRY_SENSORS          =
            RobotParams.Preferences.robotType == DriveBase.RobotType.DecodeRobot;
        public static final boolean HAS_EXIT_TRIGGER            = true;

        public static final String MOTOR_NAME                   = SUBSYSTEM_NAME + ".Motor";
        public static final MotorType MOTOR_TYPE                = MotorType.DcMotor;
        public static final boolean MOTOR_INVERTED              = false;

        public static final String LOWER_LIMIT_SWITCH_NAME      = SUBSYSTEM_NAME + ".LowerLimit";
        public static final boolean LOWER_LIMIT_SWITCH_INVERTED = false;

        public static final double DEG_PER_COUNT                = 360.0/RobotParams.MotorSpec.GOBILDA_223_ENC_PPR;
        public static final double POS_OFFSET                   = -63.0;
        public static final double ZERO_OFFSET                  = 0.0;
        public static final double ZERO_CAL_POWER               = -0.2;

        public static final double MOTOR_PID_KP                 = 0.016;
        public static final double MOTOR_PID_KI                 = 0.01;
        public static final double MOTOR_PID_KD                 = 0.0;
        public static final double MOTOR_PID_KF                 = 0.0;
        public static final double MOTOR_PID_IZONE              = 5.0;
        public static final double POS_PID_TOLERANCE            = 3.0;
        public static final boolean SOFTWARE_PID_ENABLED        = true;

        public static final String ENTRY_SENSOR1_NAME           = SUBSYSTEM_NAME + ".EntrySensor1";
        public static final String ENTRY_SENSOR2_NAME           = SUBSYSTEM_NAME + ".EntrySensor2";
        public static final String ENTRY_TRIGGER_NAME           = SUBSYSTEM_NAME + ".EntryTrigger";

        public static final double OBJECT_DISTANCE              = 120.0;    // in degrees
        public static final double MOVE_POWER                   = 1.0;
        public static final int MAX_CAPACITY                    = 3;

        public static final boolean ENTRY_TRIGGER_COLOR_MODE    = true;
        public static final double ENTRY_REFRESH_TIMEOUT        = 1.0;

        public static final double GREEN_LOW_THRESHOLD          = 100.0;
        public static final double PURPLE_LOW_THRESHOLD         = 200.0;
        public static final double PURPLE_HIGH_THRESHOLD        = 300.0;

        public static final double ENTRY_TRIGGER_LOW_COLOR_THRESHOLD    = GREEN_LOW_THRESHOLD;
        public static final double ENTRY_TRIGGER_HIGH_COLOR_THRESHOLD   = PURPLE_HIGH_THRESHOLD;
        public static final double ENTRY_TRIGGER_COLOR_SETTLING = 0.02;     // in seconds

        public static final double ENTRY_TRIGGER_LOW_DIST_THRESHOLD     = 1.0;
        public static final double ENTRY_TRIGGER_HIGH_DIST_THRESHOLD    = 65.0;
        public static final double ENTRY_TRIGGER_DIST_SETTLING  = 0.0;      // in seconds

        public static final String EXIT_TRIGGER_NAME            = SUBSYSTEM_NAME + ".ShootVelTrigger";
        public static final double EXIT_TRIGGER_LOW_THRESHOLD   = 3000.0;   // in RPM
        public static final double EXIT_TRIGGER_HIGH_THRESHOLD  = 6000.0;   // in RPM
        public static final double EXIT_TRIGGER_SETTLING        = 0.01;     // in seconds

        public static final double[] entryPresetPositions       = {0.0, 120.0, 240.0};
        public static final double[] exitPresetPositions        = {180.0, 300.0, 60.0};
    }   //class Params

    public static final TrcMotor.PidParams motorPidParams = new TrcMotor.PidParams()
        .setPidCoefficients(
            Params.MOTOR_PID_KP, Params.MOTOR_PID_KI, Params.MOTOR_PID_KD, Params.MOTOR_PID_KF, Params.MOTOR_PID_IZONE)
        .setPidControlParams(Params.POS_PID_TOLERANCE, Params.SOFTWARE_PID_ENABLED);
    public static final TrcTriggerThresholdRange.TriggerParams entryTriggerParams =
        Params.ENTRY_TRIGGER_COLOR_MODE?
            new TrcTriggerThresholdRange.TriggerParams(
                Params.ENTRY_TRIGGER_LOW_COLOR_THRESHOLD, Params.ENTRY_TRIGGER_HIGH_COLOR_THRESHOLD,
                Params.ENTRY_TRIGGER_COLOR_SETTLING):
            new TrcTriggerThresholdRange.TriggerParams(
                Params.ENTRY_TRIGGER_LOW_DIST_THRESHOLD, Params.ENTRY_TRIGGER_HIGH_DIST_THRESHOLD,
                Params.ENTRY_TRIGGER_DIST_SETTLING);
    public static final TrcTriggerThresholdRange.TriggerParams exitTriggerParams =
        new TrcTriggerThresholdRange.TriggerParams(
            Params.EXIT_TRIGGER_LOW_THRESHOLD, Params.EXIT_TRIGGER_HIGH_THRESHOLD, Params.EXIT_TRIGGER_SETTLING);

    private enum RefreshSlotState
    {
        MOVE_TO_NEXT_SLOT,
        EXAMINE_SLOT,
        DONE
    }   //enum RefreshSlotState

    private final FtcDashboard dashboard;
    private final Robot robot;
    private final TrcEvent entryTriggerEvent;
    private final RevColorSensorV3 entryAnalogSensor1;
    private final RevColorSensorV3 entryAnalogSensor2;
    private final TrcTrigger shootVelTrigger;
    public final TrcPidStorage spindexer;
    private final TrcWarpSpace warpSpace;
    private final TrcTaskMgr.TaskObject refreshSlotStatesTaskObj;
    private final TrcStateMachine<RefreshSlotState> sm;
    private final TrcEvent event;
    private final TrcTriggerThresholdRange entryTrigger;

    private final Vision.ArtifactType[] slotStates =
        {Vision.ArtifactType.None, Vision.ArtifactType.None, Vision.ArtifactType.None};
    private boolean autoReceivedEnabled = false;
    private Integer entrySlot = null;
    private Integer exitSlot = null;
    private int numPurpleArtifacts = 0;
    private int numGreenArtifacts = 0;
    private int numUnknownArtifacts = 0;
    private Vision.ArtifactType expectedArtifactType = Vision.ArtifactType.Any;
    private TrcEvent.Callback exitTriggerCallback = null;
    private int examinedSlotIndex = 0;
    private TrcEvent slotsFullEvent = null;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param robot specifies the robot object to access the other subsystems.
     */
    public Spindexer(Robot robot)
    {
        super(Params.SUBSYSTEM_NAME, Params.NEED_ZERO_CAL);
        dashboard = FtcDashboard.getInstance();
        this.robot = robot;
        FtcPidStorage.Params spindexerParams = new FtcPidStorage.Params()
            .setPrimaryMotor(Params.MOTOR_NAME, Params.MOTOR_TYPE, Params.MOTOR_INVERTED)
            .setLowerLimitSwitch(Params.LOWER_LIMIT_SWITCH_NAME, Params.LOWER_LIMIT_SWITCH_INVERTED)
            .setObjectDistance(Params.OBJECT_DISTANCE)
            .setMovePower(Params.MOVE_POWER)
            .setMaxCapacity(Params.MAX_CAPACITY);

        if (Params.HAS_ENTRY_SENSOR)
        {
            entryTriggerEvent = new TrcEvent(Params.SUBSYSTEM_NAME + "entryTrigger");
            entryAnalogSensor1 = FtcOpMode.getInstance().hardwareMap.get(
                RevColorSensorV3.class, Params.ENTRY_SENSOR1_NAME);
            if (Params.DUAL_ENTRY_SENSORS)
            {
                entryAnalogSensor2 = FtcOpMode.getInstance().hardwareMap.get(
                    RevColorSensorV3.class, Params.ENTRY_SENSOR2_NAME);
            }
            else
            {
                entryAnalogSensor2 = null;
            }
            spindexerParams.setEntryAnalogSourceTrigger(
                Params.ENTRY_TRIGGER_NAME, this::getEntrySensorValue, entryTriggerParams, false,
                this::entryTriggerCallback, entryTriggerEvent);
        }
        else
        {
            entryTriggerEvent = null;
            entryAnalogSensor1 = null;
            entryAnalogSensor2 = null;
        }

        if (Params.HAS_EXIT_TRIGGER)
        {
            shootVelTrigger = new FtcSensorTrigger()
                .setAnalogSourceTrigger(
                    Params.EXIT_TRIGGER_NAME,
                    () -> robot.shooterSubsystem != null? robot.shooterSubsystem.getFlywheelRPM(): 0.0,
                    exitTriggerParams).getTrigger();
        }
        else
        {
            shootVelTrigger = null;
        }

        spindexer = new FtcPidStorage(Params.SUBSYSTEM_NAME, spindexerParams).getPidStorage();
        spindexer.motor.setPositionSensorScaleAndOffset(Params.DEG_PER_COUNT, Params.POS_OFFSET, Params.ZERO_OFFSET);
        spindexer.motor.setPositionPidParameters(motorPidParams, null);
        warpSpace = new TrcWarpSpace(Params.SUBSYSTEM_NAME + ".warpSpace", 0.0, 360.0);
        refreshSlotStatesTaskObj = TrcTaskMgr.createTask(
            Params.SUBSYSTEM_NAME + ".refreshSlotStatesTask", this::refreshSlotStatesTask);
        sm = new TrcStateMachine<>(Params.SUBSYSTEM_NAME + ".refreshSlotStates");
        event = new TrcEvent(Params.SUBSYSTEM_NAME + ".event");
        entryTrigger = (TrcTriggerThresholdRange) spindexer.getEntryTrigger();
    }   //Spindexer

    /**
     * This method returns the created TrcRollerIntake.
     *
     * @return created Roller Intake.
     */
    public TrcPidStorage getPidStorage()
    {
        return spindexer;
    }   //getPidStorage

    /**
     * This method sets the entry trigger to color mode or distance mode.
     *
     * @param colorMode specifies true to set to color mode, false to distance mode.
     */
    private void setEntryTriggerColorMode(boolean colorMode)
    {
        if (entryTrigger != null)
        {
            if (colorMode)
            {
                entryTrigger.setTrigger(
                    Params.ENTRY_TRIGGER_LOW_COLOR_THRESHOLD, Params.ENTRY_TRIGGER_HIGH_COLOR_THRESHOLD,
                    Params.ENTRY_TRIGGER_COLOR_SETTLING);
            }
            else
            {
                entryTrigger.setTrigger(
                    Params.ENTRY_TRIGGER_LOW_DIST_THRESHOLD, Params.ENTRY_TRIGGER_HIGH_DIST_THRESHOLD,
                    Params.ENTRY_TRIGGER_DIST_SETTLING);
            }
        }
    }   //setEntryTriggerColorMode

    /**
     * This method is called when the entry sensor is triggered, it will update the entry slot state according to the
     * sensor value.
     *
     * @param context (not used).
     * @param canceled specifies true is the trigger is disabled, false otherwise.
     */
    private void entryTriggerCallback(Object context, boolean canceled)
    {
        if (!canceled && entrySlot != null)
        {
            // This gets called only if the entry trigger is enabled which is controlled by the Intake subsystem.
            // In other words, if this gets called, it is guaranteed that the Intake is active and an artifact has
            // just entered the entry slot of the Spindexer.
            TrcEvent triggerEvent = (TrcEvent) context;
            TrcDataBuffer.DataSummary triggeredData = entryTrigger.getLastTriggeredData();
            double value = triggeredData.averageValue;
            String artifactName = null;
            Vision.ArtifactType artifactType = getColorArtifactType(
                Params.ENTRY_TRIGGER_COLOR_MODE? value: getEntrySensorHue());

            if (artifactType == Vision.ArtifactType.Purple)
            {
                numPurpleArtifacts++;
                artifactName = LEDIndicator.PURPLE_BLOB;
            }
            else if (artifactType == Vision.ArtifactType.Green)
            {
                numGreenArtifacts++;
                artifactName = LEDIndicator.GREEN_BLOB;
            }
            else if (artifactType == Vision.ArtifactType.Unknown)
            {
                numUnknownArtifacts++;
                artifactName = LEDIndicator.UNKNOWN_BLOB;
            }

            slotStates[entrySlot] = artifactType;
            updateExpectedArtifactType();
            // We are done adding the entry artifact. Turn off entry trigger so we can move the Spindexer without it
            // triggering.
            spindexer.setEntryTriggerEnabled(false);
            spindexer.tracer.traceInfo(
                instanceName,
                "Entry[%d]: colorMode=%s, value=%f/%.0f, artifact=%s, slotStates=%s (PGU=%d/%d/%d), expectedNext=%s",
                entrySlot, Params.ENTRY_TRIGGER_COLOR_MODE, value, getEntrySensorHue(), artifactType,
                Arrays.toString(slotStates), numPurpleArtifacts, numGreenArtifacts, numUnknownArtifacts,
                expectedArtifactType);
            spindexer.tracer.traceDebug(
                instanceName, "value=%f (%f/%f), count=%d",
                value, triggeredData.minimumValue, triggeredData.maximumValue, triggeredData.dataCount);

            if (robot.ledIndicator != null && artifactName != null)
            {
                robot.ledIndicator.setSpindexerPatternOn(entrySlot, artifactName);
            }

            if (numPurpleArtifacts + numGreenArtifacts + numUnknownArtifacts < 3)
            {
                moveToNextVacantEntrySlot(null, triggerEvent);
                // moveToNextVacantEntrySlot will signal the triggerEvent when done.
                // Consume it here so we don't signal it twice.
                triggerEvent = null;
            }
            else if (robot.intakeSubsystem.isBulldozeEnabled())
            {
                spindexer.tracer.traceInfo(instanceName, "Slots are full! (event=" + slotsFullEvent + ")");
                if (slotsFullEvent != null)
                {
                    slotsFullEvent.signal();
                    slotsFullEvent = null;
                }
                robot.intakeSubsystem.setBulldozeIntakeEnabled(false, null, null);
            }

            if (triggerEvent != null)
            {
                triggerEvent.signal();
            }
        }
        else
        {
            spindexer.tracer.traceInfo(
                instanceName, "Entry[%s]: entrySensor=%s, canceled=%s",
                entrySlot, spindexer.isEntrySensorActive(), canceled);
        }
    }   //entryTriggerCallback

    /**
     * This method enables the Flywheel Velocity Trigger with the given threshold ranges.
     *
     * @param lowThreshold specifies the low threshold value of the range in RPM.
     * @param highThreshold specifies the high threshold value of the range in RPM.
     * @param callback specifies the method to call when the trigger occurs.
     */
    public void enableExitTrigger(double lowThreshold, double highThreshold, TrcEvent.Callback callback)
    {
        if (shootVelTrigger != null)
        {
            spindexer.tracer.traceInfo(
                instanceName,
                "Enabling exit trigger: lowThreshold=%f, highThreshold=%f", lowThreshold, highThreshold);
            exitTriggerCallback = callback;
            ((TrcTriggerThresholdRange) shootVelTrigger).setTrigger(
                lowThreshold, highThreshold, Params.EXIT_TRIGGER_SETTLING);
            shootVelTrigger.enableTrigger(TrcTrigger.TriggerMode.OnInactive, this::velTriggerCallback);
        }
    }   //enableExitTrigger

    /**
     * This method disables the Flywheel Velocity Trigger.
     */
    public void disableExitTrigger()
    {
        if (shootVelTrigger != null)
        {
            spindexer.tracer.traceInfo(instanceName, "Disabling exit trigger.");
            shootVelTrigger.disableTrigger();
        }
    }   //disableExitTrigger

    /**
     * This method is called when the shooter velocity trigger occurred, it will update the exit slot state according
     * to the sensor state.
     *
     * @param context (not used).
     * @param canceled specifies true is the trigger is disabled, false otherwise.
     */
    private void velTriggerCallback(Object context, boolean canceled)
    {
        if (!canceled)
        {
            if (exitSlot != null)
            {
                // This gets called only if the exit trigger is enabled which is controlled by the Shooter subsystem.
                // In other words, if this gets called, it is guaranteed that the Shooter is active and an artifact has
                // just left the exit slot of the Spindexer.
                double vel = ((TrcTriggerThresholdRange) shootVelTrigger).getTriggeredAverageValue();
                Vision.ArtifactType artifactType = slotStates[exitSlot];
                slotStates[exitSlot] = Vision.ArtifactType.None;
                if (artifactType == Vision.ArtifactType.Purple)
                {
                    numPurpleArtifacts--;
                }
                else if (artifactType == Vision.ArtifactType.Green)
                {
                    numGreenArtifacts--;
                }
                else if (artifactType == Vision.ArtifactType.Unknown)
                {
                    numUnknownArtifacts--;
                }
                updateExpectedArtifactType();
                // We are done removing the exit artifact. Turn off exit trigger.
                spindexer.setExitTriggerEnabled(false);
                spindexer.tracer.traceInfo(
                    instanceName,
                    "Exit[%d]: vel=%f, artifact=%s, numPurple=%d, numGreen=%d, numUnknown=%d, expectedNext=%s",
                    exitSlot, vel, artifactType, numPurpleArtifacts, numGreenArtifacts, numUnknownArtifacts,
                    expectedArtifactType);

                if (robot.ledIndicator != null)
                {
                    robot.ledIndicator.setSpindexerPatternOff(exitSlot, false);
                }

                if (exitTriggerCallback != null)
                {
                    // Shooter is on the same thread, it's safe to call back on this thread.
                    exitTriggerCallback.notify(null, false);
                    exitTriggerCallback = null;
                }
            }
            else
            {
                // If we are shooting, Spindexer better be aligned with the exit. If not, somebody did not set up
                // the Spindexer correctly.
                spindexer.tracer.traceWarn(instanceName, "Sanity check, should never come here!");
            }
        }
        else
        {
            spindexer.tracer.traceInfo(
                instanceName, "Exit[%s]: exitSensor=%s, canceled=%s",
                exitSlot, spindexer.isExitSensorActive(), canceled);
            if (exitTriggerCallback != null)
            {
                // Shooter is on the same thread, it's safe to call back on this thread.
                exitTriggerCallback.notify(null, true);
                exitTriggerCallback = null;
            }
        }
    }   //velTriggerCallback

    /**
     * This method checks the next artifact type to pick up by examining the number of purple and green artifacts
     * already in the Spindexer.
     */
    private void updateExpectedArtifactType()
    {
        if (numPurpleArtifacts + numGreenArtifacts + numUnknownArtifacts == 3)
        {
            expectedArtifactType = Vision.ArtifactType.None;
        }
        else if (numPurpleArtifacts == 2)
        {
            // We have two artifacts and both are purple.
            expectedArtifactType = Vision.ArtifactType.Green;
        }
        else if (numGreenArtifacts == 0)
        {
            // We have either one purple or no artifact at all.
            expectedArtifactType = Vision.ArtifactType.Any;
        }
        else
        {
            // We have either one purple, one green or just one green.
            expectedArtifactType = Vision.ArtifactType.Purple;
        }

        if (robot.intake != null)
        {
            robot.intakeSubsystem.setPickupArtifactType(expectedArtifactType);
        }
    }   //updateExpectedArtifactType

    /**
     * This method returns the number of artifacts with the specified type in the Spindexer.
     *
     * @return number of artifacts with the specified type in Spindexer.
     */
    public int getNumArtifacts(Vision.ArtifactType artifactType)
    {
        return artifactType == Vision.ArtifactType.Purple? numPurpleArtifacts:
               artifactType == Vision.ArtifactType.Green? numGreenArtifacts:
               artifactType == Vision.ArtifactType.Unknown? numUnknownArtifacts:
               artifactType == Vision.ArtifactType.Any?
                   numPurpleArtifacts + numGreenArtifacts + numUnknownArtifacts:
                   3 - (numPurpleArtifacts + numGreenArtifacts + numUnknownArtifacts);
    }   //getNumArtifacts

    /**
     * This method checks if the entry sensor is triggered.
     *
     * @return true if entry sensor is in triggered state, false otherwise.
     */
    public boolean isEntrySensorActive()
    {
        return spindexer.isEntrySensorActive();
    }   //isEntrySensorActive

    /**
     * This method reads the specified color sensor and convert its RGB values to HSV.
     *
     * @param sensor specifies the color sensor to read.
     * @return HSV values.
     */
    private float[] getColorSensorHSV(RevColorSensorV3 sensor)
    {
        float[] hsvValues = {0.0f, 0.0f, 0.0f};

        NormalizedRGBA normalizedColors = sensor.getNormalizedColors();
        int[] rgbValues = new int[] {
            (int) (normalizedColors.red*255),
            (int) (normalizedColors.green*255),
            (int) (normalizedColors.blue*255)};
        Color.RGBToHSV(rgbValues[0], rgbValues[1], rgbValues[2], hsvValues);
        if (hsvValues[0] == 180.0)
        {
            // If hue is 180.0, it is ambiguous (right in the middle of purple and green). Print out the raw color
            // values so we can analyze them.
            spindexer.tracer.traceDebug(
                instanceName, "%s: hsv=%s, rgb=%s",
                "ColorSensor" + (sensor == entryAnalogSensor1? 1: 2),
                Arrays.toString(hsvValues), Arrays.toString(rgbValues));
        }

        return hsvValues;
    }   //getColorSensorHSV

    /**
     * This method reads the entry REV Color Sensor and returns the Hue value.
     *
     * @return hue value.
     */
    private double getEntrySensorHue()
    {
        double hue = 0.0;

        if (entryAnalogSensor1 != null)
        {
            float[] sensor1Hsv = getColorSensorHSV(entryAnalogSensor1);
            float[] sensor2Hsv = entryAnalogSensor2 != null? getColorSensorHSV(entryAnalogSensor2): null;
            hue = sensor2Hsv == null || sensor1Hsv[2] > sensor2Hsv[2]? sensor1Hsv[0]: sensor2Hsv[0];
            if (hue >= Params.GREEN_LOW_THRESHOLD)
            {
                spindexer.tracer.traceDebug(
                    instanceName, "sensor1Hsv=%s, sensor2Hsv=%s",
                    Arrays.toString(sensor1Hsv), Arrays.toString(sensor2Hsv));
            }
        }

        return hue;
    }   //getEntrySensorHue

    /**
     * This method reads the entry REV Color Sensor and returns the Distance value in millimeter.
     *
     * @return distance value.
     */
    private double getEntrySensorDistance()
    {
        return entryAnalogSensor1 != null? entryAnalogSensor1.getDistance(DistanceUnit.MM): 0.0;
    }   //getEntrySensorDistance

    /**
     * This method reads the entry REV Color Sensor and returns the sensor value.
     *
     * @return hue value if in color mode, distance value otherwise.
     */
    private double getEntrySensorValue()
    {
        return Params.ENTRY_TRIGGER_COLOR_MODE? getEntrySensorHue(): getEntrySensorDistance();
    }   //getEntrySensorValue

    /**
     * This method checks the color sensor for the color the artifact at the entry.
     *
     * @param hue specifies the detected hue value.
     * @return detected artifact type at the entry.
     */
    public Vision.ArtifactType getColorArtifactType(double hue)
    {
        Vision.ArtifactType artifactType = null;

        if (entryAnalogSensor1 != null)
        {
            if (hue >= Params.PURPLE_LOW_THRESHOLD)
            {
                artifactType = Vision.ArtifactType.Purple;
            }
            else if (hue >= Params.GREEN_LOW_THRESHOLD)
            {
                artifactType = Vision.ArtifactType.Green;
            }
            else
            {
                artifactType = Vision.ArtifactType.Unknown;
            }
            spindexer.tracer.traceInfo(instanceName, "EntryArtifact: hue=%.3f, type=%s", hue, artifactType);
        }

        return artifactType;
    }   //getColorArtifactType

    /**
     * This method returns the artifact type in the current entry slot.
     *
     * @return artifact type in current entry slot, null if Spindexer slot is not aligned with entry.
     */
    public Vision.ArtifactType getEntrySlotArtifactType()
    {
        Vision.ArtifactType artifactType = null;

        if (entrySlot != null)
        {
            artifactType = slotStates[entrySlot];
        }

        return artifactType;
    }   //getEntrySlotArtifactType

    /**
     * This method returns the artifact type in the current exit slot.
     *
     * @return artifact type in current exit slot, null if Spindexer slot is not aligned with exit.
     */
    public Vision.ArtifactType getExitSlotArtifactType()
    {
        Vision.ArtifactType artifactType = null;

        if (exitSlot != null)
        {
            artifactType = slotStates[exitSlot];
        }

        return artifactType;
    }   //getExitSlotArtifactType

    /**
     * This method looks for the specified artifact type in the spindexer starting from the specified startSlot. It
     * will return the slot found or null if none found.
     *
     * @param artifactType specifies the artifact type to look for.
     * @param startSlot specifies the slot to start looking.
     * @return slot that contains the artifact type or null if none found.
     */
    private Integer findSlot(Vision.ArtifactType artifactType, int startSlot)
    {
        for (int i = 0; i < slotStates.length; i++)
        {
            int slot = (startSlot + i)%slotStates.length;

            if (slotStates[slot] == artifactType ||
                artifactType == Vision.ArtifactType.Any && slotStates[slot] != Vision.ArtifactType.None)
            {
                spindexer.tracer.traceInfo(
                    instanceName, "find slot for %s (%d=%s)", artifactType, slot, slotStates[slot]);
                return slot;
            }
        }

        return null;
    }   //findSlot

    /**
     * This method moves the Spindexer to the specified entry slot position.
     *
     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller is not
     *        ownership aware.
     * @param slot specifies the entry slot to move to.
     * @param event specifies the event to signal when done, null if not provided.
     */
    public void moveToEntrySlot(String owner, int slot, TrcEvent event)
    {
        spindexer.tracer.traceInfo(instanceName, "MoveToEntrySlot: Slot=%d, event=%s", slot, event);

        if (robot.shooterSubsystem != null &&
            robot.shooterSubsystem.getLauncherPosition() != Shooter.Params.LAUNCHER_REST_POS)
        {
            spindexer.tracer.traceWarn(instanceName, "Launcher is still UP, do not move!!! Let's bring it down.");
            robot.shooterSubsystem.setLauncherPosition(owner, false);
        }

        if (exitSlot != null && robot.ledIndicator != null)
        {
            robot.ledIndicator.setSpindexerPatternOff(exitSlot, true);
        }
        double pos = warpSpace.getOptimizedTarget(Params.entryPresetPositions[slot], spindexer.motor.getPosition());
        if (Params.SPIN_INTAKE_ON_MOVE)
        {
            TrcEvent callbackEvent = new TrcEvent(instanceName + ".stopIntakeCallback");
            callbackEvent.setCallback(this::stopIntakeCallback, event);
            robot.intake.intake(Intake.Params.INTAKE_POWER);
            spindexer.motor.setPosition(owner, 0.0, pos, true, Params.MOVE_POWER, callbackEvent, 0.0);
        }
        else
        {
            spindexer.motor.setPosition(owner, 0.0, pos, true, Params.MOVE_POWER, event, 0.0);
        }
        entrySlot = slot;
        exitSlot = null;
    }   //moveToEntrySlot

    /**
     * This method moves the Spindexer to the specified exit slot position.
     *
     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller is not
     *        ownership aware.
     * @param slot specifies the exit slot to move to.
     * @param event specifies the event to signal when done, null if not provided.
     */
    public void moveToExitSlot(String owner, int slot, TrcEvent event)
    {
        spindexer.tracer.traceInfo(instanceName, "MoveToExitSlot: Slot=%d, event=%s", slot, event);

        if (robot.shooterSubsystem != null &&
            robot.shooterSubsystem.getLauncherPosition() != Shooter.Params.LAUNCHER_REST_POS)
        {
            spindexer.tracer.traceWarn(instanceName, "Launcher is still UP, do not move!!! Let's bring it down.");
            robot.shooterSubsystem.setLauncherPosition(owner, false);
        }

        if (exitSlot != null && robot.ledIndicator != null)
        {
            robot.ledIndicator.setSpindexerPatternOff(exitSlot, true);
        }
        double pos = warpSpace.getOptimizedTarget(Params.exitPresetPositions[slot], spindexer.motor.getPosition());
        if (Params.SPIN_INTAKE_ON_MOVE)
        {
            TrcEvent callbackEvent = new TrcEvent(instanceName + ".stopIntakeCallback");
            callbackEvent.setCallback(this::stopIntakeCallback, event);
            robot.intake.intake(Intake.Params.INTAKE_POWER);
            spindexer.motor.setPosition(owner, 0.0, pos, true, Params.MOVE_POWER, callbackEvent, 0.0);
        }
        else
        {
            spindexer.motor.setPosition(owner, 0.0, pos, true, Params.MOVE_POWER, event, 0.0);
        }
        exitSlot = slot;
        if (robot.ledIndicator != null)
        {
            robot.ledIndicator.setSpindexerPatternOn(exitSlot, slotStates[exitSlot], true);
        }
        entrySlot = null;
    }   //moveToExitSlot

    /**
     * This method is called when the Spindexer finished spinning to the specified position to check if it needs to
     * turn off intake.
     *
     * @param context specifies the event to notify for the completion.
     * @param canceled specifies true if the event was canceled.
     */
    private void stopIntakeCallback(Object context, boolean canceled)
    {
        TrcEvent event = (TrcEvent) context;

        spindexer.tracer.traceInfo(instanceName, "stopIntakeCallback(canceled=" + canceled + ")");

        if (!canceled)
        {
            if (!robot.intakeSubsystem.isBulldozeEnabled() && !sm.isEnabled())
            {
                robot.intake.cancel();
            }

            if (event != null)
            {
                event.signal();
            }
        }
        else if (event != null)
        {
            event.cancel();
        }
    }   //stopIntakeCallback

    /**
     * This method finds the vacant slot near the current entry slot and move the spindexer to that slot position at
     * the entry. If there is no vacant slots, the spindexer will not move.
     *
     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller is not
     *        ownership aware.
     * @param event specifies the event to signal when the spindexer finishes spinning, can be null if not provided.
     * @return true if vacant slot found and spindexder spinned, false if no movement.
     */
    public boolean moveToNextVacantEntrySlot(String owner, TrcEvent event)
    {
        boolean success = false;
        Integer slot = findSlot(
            Vision.ArtifactType.None,
            entrySlot != null? entrySlot:
            exitSlot != null? (exitSlot + 1)%slotStates.length: 0);

        spindexer.tracer.traceInfo(
            instanceName, "moveToNextVacantSlot: FromSlot=" + entrySlot + ", ToSlot=" + slot);
        if (slot != null)
        {
            TrcEvent callbackEvent = new TrcEvent(instanceName + ".callbackEvent");
            callbackEvent.setCallback(this::moveEntrySlotCompletionCallback, event);
            moveToEntrySlot(owner, slot, callbackEvent);
            success = true;
        }
        else if (event != null)
        {
            // Can't find the next slot, signal completion anyway.
            event.signal();
        }

        return success;
    }   //moveToNextVacantEntrySlot

    /**
     * This method is called when the Spinidexer completed spinning to the next entry position.
     *
     * @param context not used.
     * @param canceled specifies true if canceled.
     */
    private void moveEntrySlotCompletionCallback(Object context, boolean canceled)
    {
        TrcEvent event = (TrcEvent) context;

        spindexer.tracer.traceInfo(
            instanceName,
            "moveEntrySlotCompletionCallback(autoReceive=" + autoReceivedEnabled + ", canceled=" + canceled + ")");
        if (!canceled)
        {
            if (autoReceivedEnabled)
            {
                // We are in auto receiving mode and the spindexer has finished rotating to the next vacant slot,
                // re-enable entry trigger.
                spindexer.setEntryTriggerEnabled(true);
            }

            if (event != null)
            {
                event.signal();
            }
        }
        else if (event != null)
        {
            event.cancel();
        }
    }   //moveEntrySlotCompletionCallback

    /**
     * This method finds the slot that contains the specified artifact type near the current exit slot and move the
     * spindexer to that slot position at the exit. If there is no match, the spindexer will not move.
     *
     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller is not
     *        ownership aware.
     * @param artifactType specifies the artifact type to look for.
     * @param event specifies the event to signal when the spindexer finishes spinning, can be null if not provided.
     * @return true if slot found and spindexder spinned, false if no movement.
     */
    public boolean moveToExitSlotWithArtifact(String owner, Vision.ArtifactType artifactType, TrcEvent event)
    {
        boolean success = false;
        Integer slot = findSlot(
            artifactType,
            exitSlot != null? exitSlot:
            entrySlot != null? (entrySlot + 1)%slotStates.length: 0);

        spindexer.tracer.traceInfo(
            instanceName,
            "MoveToExitSlot: FromSlot=" + exitSlot + ", ToSlot=" + slot + ", artifactType=" + artifactType);
        if (slot != null)
        {
            moveToExitSlot(owner, slot, event);
            success = true;
        }
        else if (event != null)
        {
            // Can't find the next slot, signal completion anyway.
            event.signal();
        }

        return success;
    }   //moveToExitSlotWithArtifact

    /**
     * This method move the spindexer to the next entry slot up.
     *
     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller is not
     *        ownership aware.
     * @param event specifies the event to signal when the spindexer finishes spinning, can be null if not provided.
     */
    public void entrySlotUp(String owner, TrcEvent event)
    {
        int slot = (entrySlot != null? entrySlot + 1: exitSlot + 2)%slotStates.length;
        spindexer.tracer.traceInfo(instanceName, "EntrySlotUp: FromSlot=" + entrySlot + ", ToSlot=" + slot);
        moveToEntrySlot(owner, slot, event);
    }   //entrySlotUp

    /**
     * This method move the spindexer to the next entry slot down.
     *
     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller is not
     *        ownership aware.
     * @param event specifies the event to signal when the spindexer finishes spinning, can be null if not provided.
     */
    public void entrySlotDown(String owner, TrcEvent event)
    {
        int slot = (entrySlot != null? entrySlot - 1: exitSlot + 1)%slotStates.length;
        if (slot < 0) slot += slotStates.length;
        spindexer.tracer.traceInfo(instanceName, "EntrySlotDown: FromSlot=" + entrySlot + ", ToSlot=" + slot);
        moveToEntrySlot(owner, slot, event);
    }   //entrySlotDown

    /**
     * This method move the spindexer to the next exit slot up.
     *
     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller is not
     *        ownership aware.
     * @param event specifies the event to signal when the spindexer finishes spinning, can be null if not provided.
     */
    public void exitSlotUp(String owner, TrcEvent event)
    {
        int slot = (exitSlot != null? exitSlot + 1: entrySlot + 2)%slotStates.length;
        spindexer.tracer.traceInfo(instanceName, "ExitSlotUp: FromSlot=" + exitSlot + ", ToSlot=" + slot);
        moveToExitSlot(owner, slot, event);
    }   //exitSlotUp

    /**
     * This method move the spindexer to the next exit slot down.
     *
     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller is not
     *        ownership aware.
     * @param event specifies the event to signal when the spindexer finishes spinning, can be null if not provided.
     */
    public void exitSlotDown(String owner, TrcEvent event)
    {
        int slot = (exitSlot != null? exitSlot - 1: entrySlot + 1)%slotStates.length;
        if (slot < 0) slot += slotStates.length;
        spindexer.tracer.traceInfo(instanceName, "ExitSlotDown: FromSlot=" + entrySlot + ", ToSlot=" + slot);
        moveToExitSlot(owner, slot, event);
    }   //exitSlotDown

    /**
     * This method clears the states of all Spindexer slots.
     */
    public void clearSlotStates()
    {
        for (int i = 0; i < slotStates.length; i++)
        {
            slotStates[i] = Vision.ArtifactType.None;
            if (robot.ledIndicator != null)
            {
                robot.ledIndicator.setSpindexerPatternOff(i, false);
            }
        }
        numPurpleArtifacts = numGreenArtifacts = numUnknownArtifacts = 0;
    }   //clearSlotStates

    /**
     * This method sets the preloaded artifacts to each slots of the Spindexer.
     *
     * @param preloadedArtifacts specifies an array of artifacts loaded into the slots of the Spindexer.
     */
    public void setPreloadedArtifacts(Vision.ArtifactType... preloadedArtifacts)
    {
        if (slotStates.length != preloadedArtifacts.length)
        {
            throw new IllegalArgumentException("Must provide three Artifact types.");
        }

        numPurpleArtifacts = numGreenArtifacts = numUnknownArtifacts = 0;
        for (int i = 0; i < preloadedArtifacts.length; i++)
        {
            slotStates[i] = preloadedArtifacts[i];
            switch (preloadedArtifacts[i])
            {
                case Purple:
                    numPurpleArtifacts++;
                    if (robot.ledIndicator != null)
                    {
                        robot.ledIndicator.setSpindexerPatternOff(i, false);
                        robot.ledIndicator.setSpindexerPatternOn(i, LEDIndicator.PURPLE_BLOB);
                    }
                    break;

                case Green:
                    numGreenArtifacts++;
                    if (robot.ledIndicator != null)
                    {
                        robot.ledIndicator.setSpindexerPatternOff(i, false);
                        robot.ledIndicator.setSpindexerPatternOn(i, LEDIndicator.GREEN_BLOB);
                    }
                    break;

                default:
                    throw new IllegalArgumentException("You cannot preload " + preloadedArtifacts[i] + " artifact.");
            }
        }
    }   //setPreloadedArtifacts

    /**
     * This method examines all the Spindexer slots and update their states.
     */
    public void refreshSlotStates()
    {
        if (!sm.isEnabled())
        {
            clearSlotStates();
            if (Params.SPIN_INTAKE_ON_MOVE)
            {
                robot.intake.intake(0.25);
            }
            if (!Params.ENTRY_TRIGGER_COLOR_MODE)
            {
                setEntryTriggerColorMode(true);
            }
            examinedSlotIndex = 0;
            sm.start(RefreshSlotState.MOVE_TO_NEXT_SLOT);
            refreshSlotStatesTaskObj.registerTask(TrcTaskMgr.TaskType.POST_PERIODIC_TASK);
        }
    }   //refreshSlotStates

    /**
     * This method cancels the task that refreshes the slot states.
     */
    public void cancelSlotStatesTask()
    {
        if (sm.isEnabled())
        {
            if (Params.SPIN_INTAKE_ON_MOVE)
            {
                robot.intake.cancel();
            }
            sm.stop();
            refreshSlotStatesTaskObj.unregisterTask();
            if (!Params.ENTRY_TRIGGER_COLOR_MODE)
            {
                setEntryTriggerColorMode(false);
            }
        }
    }   //cancelSlotStatesTask

    /**
     * This method enables/disables AutoReceive.
     *
     * @param context specifies true to enable AutoReceive, false to disable.
     * @param canceled specifies if the callback is canceled.
     */
    private void performAutoReceiveEnabled(Object context, boolean canceled)
    {
        if (!canceled)
        {
            boolean enabled = (boolean) context;
            spindexer.tracer.traceInfo(instanceName, "performAutoReceiveEnabled(enable=" + enabled + ")");
            autoReceivedEnabled = enabled;
            spindexer.setEntryTriggerEnabled(enabled);
            if (!enabled)
            {
                spindexer.cancel();
            }
        }
    }   //performAutoReceiveEnabled

    /**
     * This method enables/disables AutoReceive.
     *
     * @param enabled specifies true to enable AutoReceive, false to disable.
     * @param event specifies an event to notify if spindexer is full, null if not provided. Ignored if disabled.
     */
    public void setAutoReceiveEnabled(boolean enabled, TrcEvent event)
    {
        slotsFullEvent = enabled? event: null;
        if (enabled && (entrySlot == null || slotStates[entrySlot] != Vision.ArtifactType.None))
        {
            // Entry slot is not aligned or entry slot is not vacant, find the next vacant slot.
            spindexer.tracer.traceInfo(instanceName, "Entry is not at a vacant slot, find one.");
            TrcEvent callbackEvent = new TrcEvent(Params.SUBSYSTEM_NAME + ".autoReceiveEnabled");
            callbackEvent.setCallback(this::performAutoReceiveEnabled, enabled);
            moveToNextVacantEntrySlot(null, callbackEvent);
        }
        else
        {
            performAutoReceiveEnabled(enabled, false);
        }
    }   //setAutoReceiveEnabled

    /**
     * This methods is called periodically to run the task.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the competition mode (e.g. Autonomous, TeleOp, Test).
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false otherwise.
     */
    private void refreshSlotStatesTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode, boolean slowPeriodicLoop)
    {
        RefreshSlotState state = sm.checkReadyAndGetState();
        // This task assumes the caller has cleared all slot states, turned on the intake and zeroed the examine count.
        if (state != null)
        {
            spindexer.tracer.tracePreStateInfo(sm.toString(), state);
            switch (state)
            {
                case MOVE_TO_NEXT_SLOT:
                    if (examinedSlotIndex < slotStates.length)
                    {
                        // Turn off entry trigger before moving Spindexer.
                        spindexer.setEntryTriggerEnabled(false);
                        moveToEntrySlot(null, examinedSlotIndex, event);
                        sm.waitForSingleEvent(event, RefreshSlotState.EXAMINE_SLOT);
                    }
                    else
                    {
                        sm.setState(RefreshSlotState.DONE);
                    }
                    break;

                case EXAMINE_SLOT:
                    examinedSlotIndex++;
                    spindexer.setEntryTriggerEnabled(true);
                    sm.waitForSingleEvent(
                        entryTriggerEvent, RefreshSlotState.MOVE_TO_NEXT_SLOT, Params.ENTRY_REFRESH_TIMEOUT);
                    break;

                case DONE:
                default:
                    cancelSlotStatesTask();
                    break;
            }
            spindexer.tracer.tracePostStateInfo(sm.toString(), state, null);
        }
    }   //refreshSlotStatesTask

    //
    // Implements TrcSubsystem abstract methods.
    //

    /**
     * This method cancels any pending operations.
     */
    @Override
    public void cancel()
    {
        spindexer.cancel();
    }   //cancel

    /**
     * This method starts zero calibrate of the subsystem.
     *
     * @param owner specifies the owner ID to to claim subsystem ownership, can be null if ownership not required.
     * @param event specifies an event to signal when zero calibration is done, can be null if not provided.
     */
    @Override
    public void zeroCalibrate(String owner, TrcEvent event)
    {
        TrcEvent zeroCalCallbackEvent = new TrcEvent(Params.SUBSYSTEM_NAME + ".zeroCalCallback");
        zeroCalCallbackEvent.setCallback(
            (ctxt, canceled) ->
            {
                TrcEvent zeroCalEvent = (TrcEvent) ctxt;
                spindexer.tracer.traceInfo(
                    instanceName, "ZeroCalibrateCompletion: event=%s, canceled=%s", zeroCalEvent, canceled);
                if (!canceled)
                {
                    // After zero calibration, move the Spindexer to entry slot 0.
                    moveToEntrySlot(owner, 0, null);
                    if (zeroCalEvent != null) zeroCalEvent.signal();
                }
                else if (zeroCalEvent != null)
                {
                    zeroCalEvent.cancel();
                }
            }, event);
        spindexer.zeroCalibrate(owner, Params.ZERO_CAL_POWER, zeroCalCallbackEvent);
        entrySlot = 0;
        exitSlot = null;
    }   //zeroCalibrate

    /**
     * This method resets the subsystem state. Typically, this is used to retract the subsystem for turtle mode.
     */
    @Override
    public void resetState()
    {
        // Spindexer doesn't do anything in Turtle mode.
    }   //resetState

    /**
     * This method update the dashboard with the subsystem status.
     *
     * @param lineNum specifies the starting line number to print the subsystem status.
     * @param slowLoop specifies true if this is a slow loop, false otherwise.
     * @return updated line number for the next subsystem to print.
     */
    @Override
    public int updateStatus(int lineNum, boolean slowLoop)
    {
        if (RobotParams.Preferences.showSpindexerStatus)
        {
            if (slowLoop)
            {
                dashboard.displayPrintf(
                    lineNum++, "%s: pos=%.1f/%.1f, power=%.1f, current=%.1f, LimitSw=%s",
                    Params.SUBSYSTEM_NAME, spindexer.motor.getPosition(), spindexer.motor.getPidTarget(),
                    spindexer.motor.getPower(), spindexer.motor.getCurrent(),
                    spindexer.motor.isLowerLimitSwitchActive());
                dashboard.displayPrintf(
                    lineNum++, "%s: Entry(Hue/Dist/Trig)=%.3f/%.3f/%s",
                    Params.SUBSYSTEM_NAME, getEntrySensorHue(), getEntrySensorDistance(), isEntrySensorActive());
                dashboard.displayPrintf(
                    lineNum++, "%s: purple=%d, green=%d, [%s, %s, %s]",
                    Params.SUBSYSTEM_NAME, numPurpleArtifacts, numGreenArtifacts,
                    slotStates[0], slotStates[1], slotStates[2]);
            }
        }

        if (RobotParams.Preferences.showSpindexerGraph)
        {
            dashboard.putNumber("SpindexerTarget", spindexer.motor.getPidTarget());
            dashboard.putNumber("SpindexerAngle", spindexer.motor.getPosition());
        }

        return lineNum;
    }   //updateStatus

    /**
     * This method is called to update subsystem parameter to the Dashboard.
     */
    @Override
    public void updateParamsToDashboard()
    {
    }   //updateParamsToDashboard

    /**
     * This method is called to update subsystem parameters from the Dashboard.
     */
    @Override
    public void updateParamsFromDashboard()
    {
    }   //updateParamsFromDashboard

}   //class Spindexer
