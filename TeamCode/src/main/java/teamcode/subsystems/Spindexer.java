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
import teamcode.Robot;
import teamcode.vision.Vision;
import trclib.controller.TrcPidController;
import trclib.dataprocessor.TrcWarpSpace;
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
        public static final double PURPLE_LOW_THRESHOLD         = 0.0;
        public static final double PURPLE_HIGH_THRESHOLD        = 1.0;
        public static final double GREEN_LOW_THRESHOLD          = 2.0;
        public static final double GREEN_HIGH_THRESHOLD         = 3.0;

        public static final double[] entryPresetPositions       = {0.0, 120.0, 240.0};
        public static final double[] exitPresetPositions        = {180.0, 300.0, 60.0};
    }   //class Params

    private final FtcDashboard dashboard;
    private final Robot robot;
    private final RevColorSensorV3 entryAnalogSensor;
    private final RevColorSensorV3 exitAnalogSensor;
    private final TrcPidStorage spindexer;
    private final TrcWarpSpace warpSpace;

    private final Vision.ColorBlobType[] slotStates  = {null, null, null};
    private Integer entrySlot = 0;
    private Integer exitSlot = null;
    private int numPurpleArtifacts = 0;
    private int numGreenArtifacts = 0;

    /**
     * Constructor: Creates an instance of the object.
     */
    public Spindexer(Robot robot)
    {
        super(Params.SUBSYSTEM_NAME, Params.NEED_ZERO_CAL);
        dashboard = FtcDashboard.getInstance();
        this.robot = robot;
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
                false, this::entryTriggerCallback, null);
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
                false, this::exitTriggerCallback, null);
        }
        else
        {
            exitAnalogSensor = null;
        }

        spindexer = new FtcPidStorage(Params.SUBSYSTEM_NAME, spindexerParams).getPidStorage();
        spindexer.motor.setPositionPidParameters(
            Params.posPidCoeffs, Params.POS_PID_TOLERANCE, Params.SOFTWARE_PID_ENABLED);
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
        if (!canceled && spindexer.isEntrySensorActive() && entrySlot != null)
        {
            Vision.ColorBlobType artifactType = getEntryArtifactType();
            if (artifactType == Vision.ColorBlobType.Purple)
            {
                numPurpleArtifacts++;
            }
            else if (artifactType == Vision.ColorBlobType.Green)
            {
                numGreenArtifacts++;
            }
            slotStates[entrySlot] = artifactType;
            updateExpectedArtifactType();
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
        if (!canceled && !spindexer.isExitSensorActive() && exitSlot != null)
        {
            Vision.ColorBlobType artifactType = slotStates[exitSlot];
            slotStates[exitSlot] = null;
            if (artifactType == Vision.ColorBlobType.Purple)
            {
                numPurpleArtifacts--;
            }
            else if (artifactType == Vision.ColorBlobType.Green)
            {
                numGreenArtifacts--;
            }
            updateExpectedArtifactType();
        }
    }   //exitTriggerCallback

    /**
     * This method checks the next artifact type to pick up by examining the number of purple and green artifacts
     * already in the Spindexer.
     */
    private void updateExpectedArtifactType()
    {
        Vision.ColorBlobType artifactType;

        if (numPurpleArtifacts + numGreenArtifacts == 3)
        {
            artifactType = Vision.ColorBlobType.None;
        }
        else if (numPurpleArtifacts == 2)
        {
            artifactType = Vision.ColorBlobType.Green;
        }
        else
        {
            artifactType = Vision.ColorBlobType.Purple;
        }

        if (robot.intake != null)
        {
            robot.intakeSubsystem.setExpectedArtifactType(artifactType);
        }
    }   //updateExpectedArtifactType

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

    public Vision.ColorBlobType getEntryArtifactType()
    {
        Vision.ColorBlobType artifactType = null;

        if (spindexer.isEntrySensorActive())
        {
            double hue = getSensorHue(entryAnalogSensor);

            if (hue >= Params.PURPLE_LOW_THRESHOLD && hue <= Params.PURPLE_HIGH_THRESHOLD)
            {
                artifactType = Vision.ColorBlobType.Purple;
            }
            else if (hue >= Params.GREEN_LOW_THRESHOLD && hue <= Params.GREEN_HIGH_THRESHOLD)
            {
                artifactType = Vision.ColorBlobType.Green;
            }
        }

        return artifactType;
    }   //getEntryArtifactType

    /**
     * This method looks for the specified slot type in the spindexer starting from the specified startSlot. It will
     * return the slot found or null if none found.
     *
     * @param slotType specifies the slot type to look for.
     * @param startSlot specifies the slot to start looking.
     * @return slot that matches the slot type or null if none found.
     */
    private Integer findSlot(Vision.ColorBlobType slotType, int startSlot)
    {
        for (int i = 0; i < slotStates.length; i++)
        {
            int slot = (startSlot + i)%slotStates.length;

            if (slotStates[slot] == slotType)
            {
                return slot;
            }
        }

        return null;
    }   //findSlot

    /**
     * This method finds the vacant slot near the current entry slot and move the spindexer to that slot position at
     * the entry. If there is no vacant slots, the spindexer will not move.
     */
    public void moveToEntrySlot()
    {
        Integer slot = findSlot(null, entrySlot != null? entrySlot: (exitSlot + 1)%slotStates.length);

        if (slot != null)
        {
            double pos = warpSpace.getOptimizedTarget(
                Params.entryPresetPositions[slot], spindexer.motor.getPosition());
            spindexer.motor.setPosition(pos, true, Params.MOVE_POWER);
            entrySlot = slot;
        }
    }   //moveToEntrySlot

    /**
     * This method finds the slot that matches the specified slot type near the current exit slot and move the
     * spindexer to that slot position at the exit. If there is no match, the spindexer will not move.
     */
    public void moveToExitSlot(Vision.ColorBlobType slotType)
    {
        Integer slot = findSlot(slotType, exitSlot != null? exitSlot: (entrySlot + 1)%slotStates.length);

        if (slot != null)
        {
            double pos = warpSpace.getOptimizedTarget(
                Params.exitPresetPositions[slot], spindexer.motor.getPosition());
            spindexer.motor.setPosition(pos, true, Params.MOVE_POWER);
            exitSlot = slot;
        }
    }   //moveToExitSlot

//    public void moveToSlot(Vision.ColorBlobType slotType, boolean isEntry)
//    {
//        int slot = isEntry ? entrySlot : (exitSlot == null ? 0 : exitSlot);
//
//        for (int step = 1; step <= slotStates.length; step++)
//        {
//            int slotIndex = (slot + step) % slotStates.length;
//
//            // ENTRY → look for empty
//            if (isEntry && slotStates[slotIndex] == null)
//            {
//                double targetPos = Params.entryPresetPositions[slotIndex];
//                spindexer.motor.setPosition(targetPos, true, Params.MOVE_POWER);
//                entrySlot = slotIndex;
//                exitSlot = null;
//                return;
//            }
//
//            // EXIT → look for a ball of matching type
//            if (!isEntry && slotStates[slotIndex] == slotType)
//            {
//                double targetPos = Params.exitPresetPositions[slotIndex];
//                spindexer.motor.setPosition(targetPos, true, Params.MOVE_POWER);
//                exitSlot = slotIndex;
//                entrySlot = null;
//                return;
//            }
//        }
//    }   //moveToSlot
//
//    public void updateEntrySlotState(Vision.ColorBlobType artifactType)
//    {
//        if (entrySlot != null)
//        {
//            slotStates[entrySlot] = artifactType;
//        }
//    }
//
//    public void updateExitSlotState()
//    {
//        if (exitSlot != null)
//        {
//            slotStates[exitSlot] = null;
//        }
//    }
//
    /**
     * This method move the spindexer to the next entry slot up.
     */
    public void entrySlotUp()
    {
        int slot = (entrySlot != null? entrySlot + 1: exitSlot + 2)%slotStates.length;
        double pos = warpSpace.getOptimizedTarget(
            Params.entryPresetPositions[slot], spindexer.motor.getPosition());
        spindexer.motor.setPosition(pos, true, Params.MOVE_POWER);
        entrySlot = slot;
    }   //entrySlotUp

    /**
     * This method move the spindexer to the next entry slot down.
     */
    public void entrySlotDown()
    {
        int slot = (entrySlot != null? entrySlot - 1: exitSlot + 1)%slotStates.length;
        if (slot < 0) slot += slotStates.length;
        double pos = warpSpace.getOptimizedTarget(
            Params.entryPresetPositions[slot], spindexer.motor.getPosition());
        spindexer.motor.setPosition(pos, true, Params.MOVE_POWER);
        entrySlot = slot;
    }   //entrySlotDown

    /**
     * This method move the spindexer to the next exit slot up.
     */
    public void exitSlotUp()
    {
        int slot = (exitSlot != null? exitSlot + 1: entrySlot + 2)%slotStates.length;
        double pos = warpSpace.getOptimizedTarget(
            Params.exitPresetPositions[slot], spindexer.motor.getPosition());
        spindexer.motor.setPosition(pos, true, Params.MOVE_POWER);
        exitSlot = slot;
    }   //exitSlotUp

    /**
     * This method move the spindexer to the next exit slot down.
     */
    public void exitSlotDown()
    {
        int slot = (exitSlot != null? exitSlot - 1: entrySlot + 1)%slotStates.length;
        if (slot < 0) slot += slotStates.length;
        double pos = warpSpace.getOptimizedTarget(
            Params.exitPresetPositions[slot], spindexer.motor.getPosition());
        spindexer.motor.setPosition(pos, true, Params.MOVE_POWER);
        exitSlot = slot;
    }   //exitSlotDown

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
        if (slowLoop)
        {
            dashboard.displayPrintf(
                lineNum++, "%s: pos=%.3f/%.3f, LimitSw=%s, hue(entry/exit)=%f/%f",
                Params.SUBSYSTEM_NAME, spindexer.motor.getPosition(), spindexer.motor.getPidTarget(),
                spindexer.motor.isLowerLimitSwitchActive(), getEntrySensorHue(), getExitSensorHue());
            dashboard.displayPrintf(
                lineNum++, "%s: slots(%s, %s, %s)", Params.SUBSYSTEM_NAME, slotStates[0], slotStates[1], slotStates[2]);
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
