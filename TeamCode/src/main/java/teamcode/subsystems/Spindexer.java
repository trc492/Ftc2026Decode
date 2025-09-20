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

import ftclib.driverio.FtcDashboard;
import ftclib.motor.FtcMotorActuator.MotorType;
import ftclib.robotcore.FtcOpMode;
import ftclib.subsystem.FtcPidStorage;
import teamcode.Dashboard;
import teamcode.vision.Vision;
import trclib.controller.TrcPidController;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcPresets;
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

        public static final String MOTOR_NAME                   = "Motor";
        public static final MotorType MOTOR_TYPE                = MotorType.DcMotor;
        public static final boolean MOTOR_INVERTED              = true;

        public static final String LOWER_LIMIT_SWITCH_NAME      = "LowerLimit";
        public static final boolean LOWER_LIMIT_SWITCH_INVERTED = false;

        public static final double GOBILDA312_CPR               = (((1.0 + (46.0/17.0))) * (1.0 + (46.0/11.0))) * 28.0;
        public static final double DEG_PER_COUNT                = 360.0 / GOBILDA312_CPR;
        public static final double POS_OFFSET                   = 0.0;
        public static final double ZERO_OFFSET                  = 0.0;
        public static final double ZERO_CAL_POWER               = -0.2;

        public static final boolean SOFTWARE_PID_ENABLED        = true;
        public static final TrcPidController.PidCoefficients posPidCoeffs =
            new TrcPidController.PidCoefficients(1.0, 0.0, 0.0, 0.0, 0.0);
        public static final double POS_PID_TOLERANCE            = 1.0;

        public static final String ENTRY_SENSOR_NAME            = "EntrySensor";
        public static final String EXIT_SENSOR_NAME             = "ExitSensor";

        public static final double OBJECT_DISTANCE              = 120.0;    // in degrees
        public static final double MOVE_POWER                   = 1.0;
        public static final int MAX_CAPACITY                    = 3;

        public static final double ENTRY_TRIGGER_LOW_THRESHOLD  = 0.0;  // in inches
        public static final double ENTRY_TRIGGER_HIGH_THRESHOLD = 0.0;  // in inches
        public static final double ENTRY_TRIGGER_SETTLING       = 0.2;  // in seconds
        public static final double EXIT_TRIGGER_LOW_THRESHOLD   = 0.0;  // in inches
        public static final double EXIT_TRIGGER_HIGH_THRESHOLD  = 0.0;  // in inches
        public static final double EXIT_TRIGGER_SETTLING        = 0.2;  // in seconds

        public static final String ENTRY_PRESETS_NAME           = "EntryPresets";
        public static final String EXIT_PRESETS_NAME            = "ExitPresets";
        public static final double[] entryPresetPositions       = {0.0, 120.0, 240.0};
        public static final double[] exitPresetPositions        = {60.0, 180.0, 300.0};
        public static final double POS_PRESET_TOLERANCE         = 5.0;

        public static Vision.ColorBlobType[] currentSlots       = {null, null, null};
        public static Integer currentEntrySlot                  = 0;
        public static Integer currentExitSlot                   = null;


    }   //class Params

    private final FtcDashboard dashboard;
    private final RevColorSensorV3 entryAnalogSensor;
    private final RevColorSensorV3 exitAnalogSensor;
    private final TrcPidStorage spindexer;
    private final TrcPresets entryPresets;
    private final TrcPresets exitPresets;

    /**
     * Constructor: Creates an instance of the object.
     */
    public Spindexer()
    {
        super(Params.SUBSYSTEM_NAME, Params.NEED_ZERO_CAL);
        dashboard = FtcDashboard.getInstance();
        FtcPidStorage.Params spindexerParams = new FtcPidStorage.Params()
            .setPrimaryMotor(
                Params.SUBSYSTEM_NAME + "." + Params.MOTOR_NAME, Params.MOTOR_TYPE, Params.MOTOR_INVERTED)
            .setLowerLimitSwitch(
                Params.SUBSYSTEM_NAME + "." + Params.LOWER_LIMIT_SWITCH_NAME, Params.LOWER_LIMIT_SWITCH_INVERTED)
            .setPositionScaleAndOffset(Params.DEG_PER_COUNT, Params.POS_OFFSET, Params.ZERO_OFFSET)
            .setObjectDistance(Params.OBJECT_DISTANCE)
            .setMovePower(Params.MOVE_POWER)
            .setMaxCapacity(Params.MAX_CAPACITY);

        if (Params.HAS_ENTRY_SENSOR)
        {
            entryAnalogSensor = FtcOpMode.getInstance().hardwareMap.get(
                RevColorSensorV3.class, Params.SUBSYSTEM_NAME + "." + Params.ENTRY_SENSOR_NAME);
            spindexerParams.setEntryAnalogSourceTrigger(
                Params.SUBSYSTEM_NAME + "." + Params.ENTRY_SENSOR_NAME, this::getEntrySensorData,
                Params.ENTRY_TRIGGER_LOW_THRESHOLD, Params.ENTRY_TRIGGER_HIGH_THRESHOLD, Params.ENTRY_TRIGGER_SETTLING,
                false, null, null);
        }
        else
        {
            entryAnalogSensor = null;
        }

        if (Params.HAS_EXIT_SENSOR)
        {
            exitAnalogSensor = FtcOpMode.getInstance().hardwareMap.get(
                RevColorSensorV3.class, Params.SUBSYSTEM_NAME + "." + Params.EXIT_SENSOR_NAME);
            spindexerParams.setExitAnalogSourceTrigger(
                Params.SUBSYSTEM_NAME + "." + Params.EXIT_SENSOR_NAME, this::getExitSensorData,
                Params.EXIT_TRIGGER_LOW_THRESHOLD, Params.EXIT_TRIGGER_HIGH_THRESHOLD, Params.EXIT_TRIGGER_SETTLING,
                false, null, null);
        }
        else
        {
            exitAnalogSensor = null;
        }

        spindexer = new FtcPidStorage(Params.SUBSYSTEM_NAME, spindexerParams).getPidStorage();
        spindexer.motor.setPositionPidParameters(
            Params.posPidCoeffs, Params.POS_PID_TOLERANCE, Params.SOFTWARE_PID_ENABLED);
        entryPresets = new TrcPresets(
            Params.SUBSYSTEM_NAME + "." + Params.ENTRY_PRESETS_NAME, Params.POS_PRESET_TOLERANCE,
            Params.entryPresetPositions);
        exitPresets = new TrcPresets(
            Params.SUBSYSTEM_NAME + "." + Params.EXIT_PRESETS_NAME, Params.POS_PRESET_TOLERANCE,
            Params.exitPresetPositions);
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
     * This method is called by the entry sensor trigger to monitor the sensor value for triggering condition.
     *
     * @return entry sensor value.
     */
    private double getEntrySensorData()
    {
        return entryAnalogSensor != null? entryAnalogSensor.getDistance(DistanceUnit.INCH): 0.0;
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
     * This method reads the REV Color Sensor and returns the Hue value.
     *
     * @param sensor specifies the REV color sensor object.
     * @return hue value.
     */
    private double getSensorHue(RevColorSensorV3 sensor)
    {
        if (sensor != null)
        {
            float[] hsvValues = {0.0f, 0.0f, 0.0f};
            NormalizedRGBA normalizedColors = sensor.getNormalizedColors();
            Color.RGBToHSV(
                (int) (normalizedColors.red*255),
                (int) (normalizedColors.green*255),
                (int) (normalizedColors.blue*255),
                hsvValues);
            return hsvValues[0];
        }

        return 0.0;
    }   //getSensorHue

    /**
     * This method returns the hue value read from the entry sensor.
     *
     * @return hue value.
     */
    public double getEntrySensorHue()
    {
        return getSensorHue(entryAnalogSensor);
    }   //getEntrySensorHue

    /**
     * This method returns the hue value read from the exit sensor.
     *
     * @return hue value.
     */
    public double getExitSensorHue()
    {
        return getSensorHue(exitAnalogSensor);
    }   //getExitSensorHue

    public void moveToSlot(Vision.ColorBlobType slotType, boolean isEntry) {
        int currentIndex = isEntry ? Params.currentEntrySlot :
                (Params.currentExitSlot == null ? 0 : Params.currentExitSlot);

        for (int step = 1; step <= Params.currentSlots.length; step++) {
            int slotIndex = (currentIndex + step) % Params.currentSlots.length;

            // ENTRY → look for empty
            if (isEntry && Params.currentSlots[slotIndex] == null) {
                double targetPos = Params.entryPresetPositions[slotIndex];
                spindexer.motor.setPosition(targetPos, true, Params.MOVE_POWER);
                Params.currentEntrySlot = slotIndex;
                Params.currentExitSlot = null;
                return;
            }

            // EXIT → look for a ball of matching type
            if (!isEntry && Params.currentSlots[slotIndex] == slotType) {
                double targetPos = Params.exitPresetPositions[slotIndex];
                spindexer.motor.setPosition(targetPos, true, Params.MOVE_POWER);
                Params.currentExitSlot = slotIndex;
                Params.currentEntrySlot = null;
                return;
            }
        }
        return;
    }



    /**
     * This method rotates the Spindexer to the next vacant slot at the entrance. If there is no vacant slot, the
     * Spindexer will not turn.
     */
    public void setEntryPosition()
    {
        moveToSlot(null, true);
    }   //setEntryPosition

    /**
     * This method rotates the Spindexer to the slot containing the specified artifact type. If the specified artifact
     * type is not found, the Spindexer will not turn.
     *
     * @param artifactType specifies the artifact type.
     */
    public void setExitPosition(Vision.ColorBlobType artifactType)
    {

        moveToSlot(artifactType, false);
    }   //setExitPosition

    public void updateEntrySlot(Vision.ColorBlobType ballType) {
        if (Params.currentEntrySlot != null) {
            Params.currentSlots[Params.currentEntrySlot] = ballType;
        }

    }

    public void updateExitSlot() {
        if (Params.currentExitSlot != null) {
            Params.currentSlots[Params.currentExitSlot] = null;
        }
    }

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
        spindexer.zeroCalibrate(owner, Params.ZERO_CAL_POWER, event);
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
        if (slowLoop)
        {
            dashboard.displayPrintf(
                lineNum++, "%s: pos=%.3f/%.3f, LimitSw=%s, hue(entry/exit)=%f/%f",
                Params.SUBSYSTEM_NAME, spindexer.motor.getPosition(), spindexer.motor.getPidTarget(),
                spindexer.motor.isLowerLimitSwitchActive(), getEntrySensorHue(), getExitSensorHue());
        }

        return lineNum;
    }   //updateStatus

    /**
     * This method is called to initialize the Dashboard from subsystem parameters.
     *
     * @param subComponent specifies the sub-component of the Subsystem to be tuned, can be null if no sub-component.
     */
    @Override
    public void initDashboardFromSubsystemParams(String subComponent)
    {
        if (subComponent != null)
        {
            if (subComponent.equalsIgnoreCase(Params.MOTOR_NAME))
            {
                Dashboard.PidTuning.pidCoeffs = Params.posPidCoeffs;
                Dashboard.PidTuning.pidTolerance = Params.POS_PID_TOLERANCE;
                Dashboard.PidTuning.useSoftwarePid = Params.SOFTWARE_PID_ENABLED;
            }
            else if (subComponent.equalsIgnoreCase(Params.ENTRY_SENSOR_NAME))
            {
                Dashboard.TriggerThresholdsTuning.lowThreshold = Params.ENTRY_TRIGGER_LOW_THRESHOLD;
                Dashboard.TriggerThresholdsTuning.highThreshold = Params.ENTRY_TRIGGER_HIGH_THRESHOLD;
                Dashboard.TriggerThresholdsTuning.settlingPeriod = Params.ENTRY_TRIGGER_SETTLING;
            }
            else if (subComponent.equalsIgnoreCase(Params.EXIT_SENSOR_NAME))
            {
                Dashboard.TriggerThresholdsTuning.lowThreshold = Params.EXIT_TRIGGER_LOW_THRESHOLD;
                Dashboard.TriggerThresholdsTuning.highThreshold = Params.EXIT_TRIGGER_HIGH_THRESHOLD;
                Dashboard.TriggerThresholdsTuning.settlingPeriod = Params.EXIT_TRIGGER_SETTLING;
            }
        }
    }   //initDashboardFromSubsystemParams

    /**
     * This method is called to initialize the subsystem parameters from the Dashboard for tuning.
     *
     * @param subComponent specifies the sub-component of the Subsystem to be tuned, can be null if no sub-component.
     */
    @Override
    public void initSubsystemParamsForTuning(String subComponent)
    {
        if (subComponent != null)
        {
            if (subComponent.equalsIgnoreCase(Params.MOTOR_NAME))
            {
                spindexer.motor.setPositionPidParameters(
                    Dashboard.PidTuning.pidCoeffs,
                    Dashboard.PidTuning.pidTolerance,
                    Dashboard.PidTuning.useSoftwarePid);
            }
            else if (subComponent.equalsIgnoreCase(Params.ENTRY_SENSOR_NAME))
            {
                ((TrcTriggerThresholdRange) spindexer.getEntryTrigger()).setTrigger(
                    Dashboard.TriggerThresholdsTuning.lowThreshold,
                    Dashboard.TriggerThresholdsTuning.highThreshold,
                    Dashboard.TriggerThresholdsTuning.settlingPeriod);
            }
            else if (subComponent.equalsIgnoreCase(Params.EXIT_SENSOR_NAME))
            {
                ((TrcTriggerThresholdRange) spindexer.getExitTrigger()).setTrigger(
                    Dashboard.TriggerThresholdsTuning.lowThreshold,
                    Dashboard.TriggerThresholdsTuning.highThreshold,
                    Dashboard.TriggerThresholdsTuning.settlingPeriod);
            }
        }
    }   //initSubsystemParamsForTuning

}   //class Spindexer
