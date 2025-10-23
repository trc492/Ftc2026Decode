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

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import ftclib.driverio.FtcDashboard;
import ftclib.motor.FtcMotorActuator.MotorType;
import ftclib.robotcore.FtcOpMode;
import ftclib.subsystem.FtcPidStorage;
import teamcode.Robot;
import teamcode.RobotParams;
import teamcode.vision.Vision;
import trclib.dataprocessor.TrcWarpSpace;
import trclib.motor.TrcMotor;
import trclib.robotcore.TrcEvent;
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

        public static final boolean HAS_ENTRY_SENSOR            = true;
        public static final boolean HAS_EXIT_SENSOR             = true;

        public static final String MOTOR_NAME                   = SUBSYSTEM_NAME + ".Motor";
        public static final MotorType MOTOR_TYPE                = MotorType.DcMotor;
        public static final boolean MOTOR_INVERTED              = true;

        public static final String LOWER_LIMIT_SWITCH_NAME      = SUBSYSTEM_NAME + ".LowerLimit";
        public static final boolean LOWER_LIMIT_SWITCH_INVERTED = false;

        public static final double GEAR_RATIO                   = 36.0/28.0;    // Load to Motor
        public static final double DEG_PER_COUNT                =
            360.0/(RobotParams.MotorSpec.REV_COREHEX_ENC_PPR*GEAR_RATIO);
        public static final double POS_OFFSET                   = 25.0;
        public static final double ZERO_OFFSET                  = 0.0;
        public static final double ZERO_CAL_POWER               = 0.5;

        public static final double MOTOR_PID_KP                 = 0.036;
        public static final double MOTOR_PID_KI                 = 0.0;
        public static final double MOTOR_PID_KD                 = 0.0;
        public static final double POS_PID_TOLERANCE            = 5.0;
        public static final boolean SOFTWARE_PID_ENABLED        = true;

        public static final String ENTRY_SENSOR_NAME            = SUBSYSTEM_NAME + ".EntrySensor";
        public static final String EXIT_SENSOR_NAME             = SUBSYSTEM_NAME + ".ExitSensor";

        public static final double OBJECT_DISTANCE              = 120.0;    // in degrees
        public static final double MOVE_POWER                   = 1.0;
        public static final int MAX_CAPACITY                    = 3;

        public static final double ENTRY_TRIGGER_LOW_THRESHOLD  = 0.0;  // in inches
        public static final double ENTRY_TRIGGER_HIGH_THRESHOLD = 1.0;  // in inches
        public static final double ENTRY_TRIGGER_SETTLING       = 0.1;  // in seconds

        public static final double EXIT_TRIGGER_LOW_THRESHOLD   = 0.0;  // in inches
        public static final double EXIT_TRIGGER_HIGH_THRESHOLD  = 4.0;  // in inches
        public static final double EXIT_TRIGGER_SETTLING        = 0.1;  // in seconds

        public static final double PURPLE_LOW_THRESHOLD         = 210.0;
        public static final double PURPLE_HIGH_THRESHOLD        = 240.0;

        public static final double GREEN_LOW_THRESHOLD          = 120.0;
        public static final double GREEN_HIGH_THRESHOLD         = 160.0;

        public static final double[] entryPresetPositions       = {0.0, 120.0, 240.0};
        public static final double[] exitPresetPositions        = {180.0, 300.0, 60.0};
    }   //class Params

    public static final TrcMotor.TuneParams motorPidParams = new TrcMotor.TuneParams()
        .setPidCoefficients(Params.MOTOR_PID_KP, Params.MOTOR_PID_KI, Params.MOTOR_PID_KD)
        .setPidParams(Params.POS_PID_TOLERANCE, Params.SOFTWARE_PID_ENABLED);
    public static final TrcTriggerThresholdRange.TriggerParams entryTriggerParams =
        new TrcTriggerThresholdRange.TriggerParams(
            Params.ENTRY_TRIGGER_LOW_THRESHOLD, Params.ENTRY_TRIGGER_HIGH_THRESHOLD, Params.ENTRY_TRIGGER_SETTLING);
    public static final TrcTriggerThresholdRange.TriggerParams exitTriggerParams =
        new TrcTriggerThresholdRange.TriggerParams(
            Params.EXIT_TRIGGER_LOW_THRESHOLD, Params.EXIT_TRIGGER_HIGH_THRESHOLD, Params.EXIT_TRIGGER_SETTLING);

    private final FtcDashboard dashboard;
    private final Robot robot;
    private final RevColorSensorV3 entryAnalogSensor;
    private final Rev2mDistanceSensor exitAnalogSensor;
    public final TrcPidStorage spindexer;
    private final TrcWarpSpace warpSpace;

    private final Vision.ArtifactType[] slotStates  = {null, null, null};
    private boolean autoReceivedEnabled = false;
    private Integer entrySlot = 0;
    private Integer exitSlot = null;
    private int numPurpleArtifacts = 0;
    private int numGreenArtifacts = 0;
    private Vision.ArtifactType expectedArtifactType = Vision.ArtifactType.Any;
    private double entrySensorDistance = 10.0;
    private double entrySensorHue = 0.0;
    private TrcEvent zeroCalEvent = null;

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
            entryAnalogSensor = FtcOpMode.getInstance().hardwareMap.get(
                RevColorSensorV3.class, Params.ENTRY_SENSOR_NAME);
            spindexerParams.setEntryAnalogSourceTrigger(
                Params.ENTRY_SENSOR_NAME, this::getEntrySensorData, entryTriggerParams.lowThreshold,
                entryTriggerParams.highThreshold, entryTriggerParams.settlingPeriod, false,
                this::entryTriggerCallback, null);
        }
        else
        {
            entryAnalogSensor = null;
        }

        if (Params.HAS_EXIT_SENSOR)
        {
            exitAnalogSensor = FtcOpMode.getInstance().hardwareMap.get(
                Rev2mDistanceSensor.class, Params.EXIT_SENSOR_NAME);
            spindexerParams.setExitAnalogSourceTrigger(
                Params.EXIT_SENSOR_NAME, this::getExitSensorData, exitTriggerParams.lowThreshold,
                exitTriggerParams.highThreshold, exitTriggerParams.settlingPeriod, false,
                this::exitTriggerCallback, null);
        }
        else
        {
            exitAnalogSensor = null;
        }

        spindexer = new FtcPidStorage(Params.SUBSYSTEM_NAME, spindexerParams).getPidStorage();
        spindexer.motor.setPositionSensorScaleAndOffset(Params.DEG_PER_COUNT, Params.POS_OFFSET, Params.ZERO_OFFSET);
        spindexer.motor.setPositionPidParameters(
            motorPidParams.pidCoeffs, motorPidParams.pidTolerance, motorPidParams.useSoftwarePid, null);
        warpSpace = new TrcWarpSpace(Params.SUBSYSTEM_NAME + ".warpSpace", 0.0, 360.0);
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
            Vision.ArtifactType artifactType = getEntryArtifactType();
            String artifactName;

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
            else
            {
                artifactName = null;
            }

            slotStates[entrySlot] = artifactType;
            updateExpectedArtifactType();
            // We are done adding the entry artifact. Turn off entry trigger so we can move the Spindexer without it
            // triggering.
            spindexer.setEntryTriggerEnabled(false);
            entrySensorDistance = 6.0;
            entrySensorHue = 0.0;
            spindexer.tracer.traceInfo(
                instanceName, "Entry[%d]: artifact=%s, numPurple=%d, numGreen=%d, expectedNext=%s",
                entrySlot, artifactType, numPurpleArtifacts, numGreenArtifacts, expectedArtifactType);

            if (robot.ledIndicator != null)
            {
                robot.ledIndicator.setSpindexerPattern(entrySlot, artifactName);
            }

            moveToNextVacantEntrySlot(instanceName, null);
        }
        else
        {
            spindexer.tracer.traceInfo(
                instanceName, "Entry[%s]: entrySensor=%s, canceled=%s",
                entrySlot, spindexer.isEntrySensorActive(), canceled);
        }
    }   //entryTriggerCallback

    /**
     * This method is called when the exit sensor is triggered, it will update the exit slot state according to the
     * sensor state.
     *
     * @param context (not used).
     * @param canceled specifies true is the trigger is disabled, false otherwise.
     */
    private void exitTriggerCallback(Object context, boolean canceled)
    {
        if (!canceled && exitSlot != null)
        {
            // This gets called only if the exit trigger is enabled which is controlled by the Shooter subsystem.
            // In other words, if this gets called, it is guaranteed that the Shooter is active and an artifact has
            // just left the exit slot of the Spindexer.
            Vision.ArtifactType artifactType = slotStates[exitSlot];
            slotStates[exitSlot] = null;
            if (artifactType == Vision.ArtifactType.Purple)
            {
                numPurpleArtifacts--;
            }
            else if (artifactType == Vision.ArtifactType.Green)
            {
                numGreenArtifacts--;
            }
            updateExpectedArtifactType();
            // We are done removing the exit artifact. Turn off exit trigger.
            spindexer.setExitTriggerEnabled(false);
            spindexer.tracer.traceInfo(
                instanceName, "Exit[%d]: artifact=%s, numPurple=%d, numGreen=%d, expectedNext=%s",
                exitSlot, artifactType, numPurpleArtifacts, numGreenArtifacts, expectedArtifactType);

            if (robot.ledIndicator != null)
            {
                robot.ledIndicator.setSpindexerPattern(exitSlot, LEDIndicator.OFF_PATTERN);
            }
        }
        else
        {
            spindexer.tracer.traceInfo(
                instanceName, "Exit[%s]: exitSensor=%s, canceled=%s",
                exitSlot, spindexer.isExitSensorActive(), canceled);
        }
    }   //exitTriggerCallback

    /**
     * This method checks the next artifact type to pick up by examining the number of purple and green artifacts
     * already in the Spindexer.
     */
    private void updateExpectedArtifactType()
    {
        if (numPurpleArtifacts + numGreenArtifacts == 3)
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
               artifactType == Vision.ArtifactType.Any? numPurpleArtifacts + numGreenArtifacts:
                   3 - (numPurpleArtifacts + numGreenArtifacts);
    }   //getNumArtifacts

    /**
     * This method is called by the entry sensor trigger to monitor the sensor value for triggering condition.
     *
     * @return entry sensor value.
     */
    public double getEntrySensorData()
    {
        if (entryAnalogSensor != null)
        {
            double distance = entryAnalogSensor.getDistance(DistanceUnit.INCH);
            double hue = getEntrySensorHue();

            if (distance < 6.0f)
            {
                // Since getEntrySensorData is called periodically, use it to update hue value as well.
                entrySensorDistance = distance;
                entrySensorHue = hue;
            }
            else
            {
                spindexer.tracer.traceDebug(
                    instanceName,
                    "Invalid data, use previous values: Distance(sensor=%f, prev=%f), Hue(sensor=%f, prev=%f)",
                    distance, entrySensorDistance, hue, entrySensorHue);
            }

            return entrySensorDistance;
        }

        return 0.0;
    }   //getEntrySensorData

    /**
     * This method is called by the exit sensor trigger to monitor the sensor value for triggering condition.
     *
     * @return entry sensor value.
     */
    private double getExitSensorData()
    {
        return exitAnalogSensor != null? exitAnalogSensor.getDistance(DistanceUnit.INCH): 0.0;
    }   //getExitSensorData

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
     * This method checks if the exit sensor is triggered.
     *
     * @return true if exit sensor is in triggered state, false otherwise.
     */
    public boolean isExitSensorActive()
    {
        return spindexer.isExitSensorActive();
    }   //isExitSensorActive

    /**
     * This method reads the entry REV Color Sensor and returns the Hue value.
     *
     * @return hue value.
     */
    private double getEntrySensorHue()
    {
        double hue = 0.0;

        if (entryAnalogSensor != null)
        {
            float[] hsvValues = {0.0f, 0.0f, 0.0f};
            NormalizedRGBA normalizedColors = entryAnalogSensor.getNormalizedColors();
            Color.RGBToHSV(
                (int) (normalizedColors.red*255),
                (int) (normalizedColors.green*255),
                (int) (normalizedColors.blue*255),
                hsvValues);

            double sensorDistance = entryAnalogSensor.getDistance(DistanceUnit.INCH);
            if (sensorDistance < 6.0f)
            {
                hue = hsvValues[0];
                entrySensorDistance = sensorDistance;
                entrySensorHue = hue;
                spindexer.tracer.traceDebug(instanceName, "Entry: distance=%f, hue=%f", sensorDistance, hue);
            }
            else
            {
                // When distance is 6.0f, hue value is invalid, use previous detected hue value instead.
                String sensorName;
                double distance;
                distance = entrySensorDistance;
                hue = entrySensorHue;
                spindexer.tracer.traceDebug(
                    instanceName,
                    "Invalid entry sensor data, use previous values. " +
                    "Distance(sensor=%f, prev=%f), Hue(sensor=%f, prev=%f)",
                    sensorDistance, distance, hsvValues[0], hue);
            }
        }

        return hue;
    }   //getEntrySensorHue

    /**
     * This method checks the color sensor for the color the artifact at the entry.
     *
     * @return detected artifact type at the entry.
     */
    public Vision.ArtifactType getEntryArtifactType()
    {
        Vision.ArtifactType artifactType = null;

        if (entryAnalogSensor != null)
        {
            double hue = getEntrySensorHue();

            if (hue >= Params.PURPLE_LOW_THRESHOLD && hue <= Params.PURPLE_HIGH_THRESHOLD)
            {
                artifactType = Vision.ArtifactType.Purple;
            }
            else if (hue >= Params.GREEN_LOW_THRESHOLD && hue <= Params.GREEN_HIGH_THRESHOLD)
            {
                artifactType = Vision.ArtifactType.Green;
            }
            spindexer.tracer.traceInfo(instanceName, "EntryArtifact: hue=%.3f, type=%s", hue, artifactType);
        }

        return artifactType;
    }   //getEntryArtifactType

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
                return slot;
            }
        }

        return null;
    }   //findSlot

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
            Vision.ArtifactType.None, entrySlot != null? entrySlot: (exitSlot + 1)%slotStates.length);

        spindexer.tracer.traceInfo(
            instanceName, "moveToNextVacantSlot: FromSlot=" + entrySlot + ", ToSlot=" + slot);
        if (slot != null)
        {
            double pos = warpSpace.getOptimizedTarget(
                Params.entryPresetPositions[slot], spindexer.motor.getPosition());
            TrcEvent callbackEvent = new TrcEvent(instanceName + ".callbackEvent");
            callbackEvent.setCallback(this::spinCompletionCallback, event);
            spindexer.motor.setPosition(owner, 0.0, pos, true, Params.MOVE_POWER, callbackEvent, 0.0);
            entrySlot = slot;
            success = true;
        }

        return success;
    }   //moveToNextVacantEntrySlot

    /**
     * This method is called when the Spinidexer completed spinning to the next position.
     *
     * @param context not used.
     * @param canceled specifies true if canceled.
     */
    private void spinCompletionCallback(Object context, boolean canceled)
    {
        TrcEvent event = (TrcEvent) context;

        spindexer.tracer.traceInfo(
            instanceName,
            "spinCompletionCallback(autoReceive=" + autoReceivedEnabled + ", canceled=" + canceled + ")");
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
    }   //spinCompletionCallback

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
        Integer slot = findSlot(artifactType, exitSlot != null? exitSlot: (entrySlot + 1)%slotStates.length);

        spindexer.tracer.traceInfo(
            instanceName,
            "MoveToExitSlot: FromSlot=" + exitSlot + ", ToSlot=" + slot + ", artifactType=" + artifactType);
        if (slot != null)
        {
            double pos = warpSpace.getOptimizedTarget(
                Params.exitPresetPositions[slot], spindexer.motor.getPosition());
            spindexer.motor.setPosition(owner, 0.0, pos, true, Params.MOVE_POWER, event, 0.0);
            exitSlot = slot;
            success = true;
        }

        return success;
    }   //moveToExitSlotWithArtifact

    /**
     * This method move the spindexer to the next entry slot up.
     *
     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller is not
     *        ownership aware.
     */
    public void entrySlotUp(String owner)
    {
        int slot = (entrySlot != null? entrySlot + 1: exitSlot + 2)%slotStates.length;
        double pos = warpSpace.getOptimizedTarget(
            Params.entryPresetPositions[slot], spindexer.motor.getPosition());

        spindexer.tracer.traceInfo(
            instanceName, "EntrySlotUp: FromSlot=" + entrySlot + ", ToSlot=" + slot + ", pos=" + pos);
        spindexer.motor.setPosition(owner, 0.0, pos, true, Params.MOVE_POWER, null, 0.0);
        entrySlot = slot;
    }   //entrySlotUp

    /**
     * This method move the spindexer to the next entry slot down.
     *
     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller is not
     *        ownership aware.
     */
    public void entrySlotDown(String owner)
    {
        int slot = (entrySlot != null? entrySlot - 1: exitSlot + 1)%slotStates.length;
        if (slot < 0) slot += slotStates.length;
        double pos = warpSpace.getOptimizedTarget(
            Params.entryPresetPositions[slot], spindexer.motor.getPosition());

        spindexer.tracer.traceInfo(
            instanceName, "EntrySlotDown: FromSlot=" + entrySlot + ", ToSlot=" + slot + ", pos=" + pos);
        spindexer.motor.setPosition(owner, 0.0, pos, true, Params.MOVE_POWER, null, 0.0);
        entrySlot = slot;
    }   //entrySlotDown

    /**
     * This method move the spindexer to the next exit slot up.
     *
     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller is not
     *        ownership aware.
     */
    public void exitSlotUp(String owner)
    {
        int slot = (exitSlot != null? exitSlot + 1: entrySlot + 2)%slotStates.length;
        double pos = warpSpace.getOptimizedTarget(
            Params.exitPresetPositions[slot], spindexer.motor.getPosition());

        spindexer.tracer.traceInfo(
            instanceName, "ExitSlotUp: FromSlot=" + exitSlot + ", ToSlot=" + slot + ", pos=" + pos);
        spindexer.motor.setPosition(owner, 0.0, pos, true, Params.MOVE_POWER, null, 0.0);
        exitSlot = slot;
    }   //exitSlotUp

    /**
     * This method move the spindexer to the next exit slot down.
     *
     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller is not
     *        ownership aware.
     */
    public void exitSlotDown(String owner)
    {
        int slot = (exitSlot != null? exitSlot - 1: entrySlot + 1)%slotStates.length;
        if (slot < 0) slot += slotStates.length;
        double pos = warpSpace.getOptimizedTarget(
            Params.exitPresetPositions[slot], spindexer.motor.getPosition());

        spindexer.tracer.traceInfo(
            instanceName, "ExitSlotDown: FromSlot=" + entrySlot + ", ToSlot=" + slot + ", pos=" + pos);
        spindexer.motor.setPosition(owner, 0.0, pos, true, Params.MOVE_POWER, null, 0.0);
        exitSlot = slot;
    }   //exitSlotDown

    /**
     * This method enables/disables AutoReceive.
     *
     * @param enabled specifies true to enable AutoReceive, false to disable.
     */
    public void setAutoReceiveEnabled(boolean enabled)
    {
        spindexer.tracer.traceInfo(instanceName, "setAutoReceiveEnabled(enable=" + enabled + ")");
        autoReceivedEnabled = enabled;
        spindexer.setEntryTriggerEnabled(enabled);
        if (!enabled)
        {
            spindexer.cancel();
        }
    }   //setAutoReceiveEnabled

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
        zeroCalEvent = event;
        TrcEvent calCompletionEvent = new TrcEvent(Params.SUBSYSTEM_NAME + ".zeroCal");
        calCompletionEvent.setCallback(
            (ctxt, canceled) ->
            {
                if (!canceled)
                {
                    // After zero calibration, move the Spindexer to entry slot 0.
                    spindexer.motor.setPosition(
                        owner, 0.0, Params.entryPresetPositions[0], true, Params.MOVE_POWER, null, 0.0);
                    if (zeroCalEvent != null) zeroCalEvent.signal();
                }
                else if (zeroCalEvent != null)
                {
                    zeroCalEvent.cancel();
                }
                zeroCalEvent = null;
            }, null);
        spindexer.zeroCalibrate(owner, Params.ZERO_CAL_POWER, calCompletionEvent);
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
                    lineNum++, "%s: Entry(Hue/Dist/Trig)=%.3f/%.3f/%s, Exit(Dist/Trig)=%.3f/%s",
                    Params.SUBSYSTEM_NAME, getEntrySensorHue(), getEntrySensorData(), isEntrySensorActive(),
                    getExitSensorData(), isExitSensorActive());
                dashboard.displayPrintf(
                    lineNum++, "%s: purple=%d, green=%d, [%s, %s, %s]",
                    Params.SUBSYSTEM_NAME, numPurpleArtifacts, numGreenArtifacts,
                    slotStates[0], slotStates[1], slotStates[2]);
            }
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
        ((TrcTriggerThresholdRange) spindexer.getEntryTrigger()).setTrigger(
            entryTriggerParams.lowThreshold, entryTriggerParams.highThreshold,
            entryTriggerParams.settlingPeriod);
        ((TrcTriggerThresholdRange) spindexer.getExitTrigger()).setTrigger(
            exitTriggerParams.lowThreshold, exitTriggerParams.highThreshold, exitTriggerParams.settlingPeriod);
    }   //updateParamsFromDashboard

}   //class Spindexer
