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

import com.qualcomm.hardware.rev.RevColorSensorV3;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import ftclib.driverio.FtcDashboard;
import ftclib.motor.FtcMotorActuator.MotorType;
import ftclib.robotcore.FtcOpMode;
import ftclib.subsystem.FtcPidStorage;
import teamcode.vision.Vision;
import trclib.controller.TrcPidController;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcPresets;
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

        public static final String MOTOR_NAME                   = SUBSYSTEM_NAME + ".motor";
        public static final MotorType MOTOR_TYPE                = MotorType.DcMotor;
        public static final boolean MOTOR_INVERTED              = true;

        public static final String LOWER_LIMIT_SWITCH_NAME      = SUBSYSTEM_NAME + ".lowerLimit";
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

        public static final String ENTRY_SENSOR_NAME            = SUBSYSTEM_NAME + ".entrySensor";
        public static final String EXIT_SENSOR_NAME             = SUBSYSTEM_NAME + ".exitSensor";

        public static final double OBJECT_DISTANCE              = 120.0;    // in degrees
        public static final double MOVE_POWER                   = 1.0;
        public static final int MAX_CAPACITY                    = 3;

        public static final double ENTRY_TRIGGER_LOW_THRESHOLD  = 0.0;  // in inches
        public static final double ENTRY_TRIGGER_HIGH_THRESHOLD = 0.0;  // in inches
        public static final double ENTRY_TRIGGER_SETTLING       = 0.2;  // in seconds
        public static final double EXIT_TRIGGER_LOW_THRESHOLD   = 0.0;  // in inches
        public static final double EXIT_TRIGGER_HIGH_THRESHOLD  = 0.0;  // in inches
        public static final double EXIT_TRIGGER_SETTLING        = 0.2;  // in seconds

        public static final String ENTRY_PRESETS_NAME           = SUBSYSTEM_NAME + ".entryPresets";
        public static final String EXIT_PRESETS_NAME            = SUBSYSTEM_NAME + ".exitPresets";
        public static final double[] entryPresetPositions       = {0.0, 120.0, 240.0};
        public static final double[] exitPresetPositions        = {60.0, 180.0, 300.0};
        public static final double POS_PRESET_TOLERANCE         = 5.0;
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
            .setPrimaryMotor(Params.MOTOR_NAME, Spindexer.Params.MOTOR_TYPE, Spindexer.Params.MOTOR_INVERTED)
            .setLowerLimitSwitch(Params.LOWER_LIMIT_SWITCH_NAME, Params.LOWER_LIMIT_SWITCH_INVERTED)
            .setPositionScaleAndOffset(Params.DEG_PER_COUNT, Params.POS_OFFSET, Params.ZERO_OFFSET)
            .setObjectDistance(Params.OBJECT_DISTANCE)
            .setMovePower(Params.MOVE_POWER)
            .setMaxCapacity(Params.MAX_CAPACITY);

        if (Params.HAS_ENTRY_SENSOR)
        {
            entryAnalogSensor = FtcOpMode.getInstance().hardwareMap.get(
                RevColorSensorV3.class, Params.ENTRY_SENSOR_NAME);
            spindexerParams.setEntryAnalogSourceTrigger(
                Params.ENTRY_SENSOR_NAME, this::getEntrySensorData, Params.ENTRY_TRIGGER_LOW_THRESHOLD,
                Params.ENTRY_TRIGGER_HIGH_THRESHOLD, Params.ENTRY_TRIGGER_SETTLING, false, null, null);
        }
        else
        {
            entryAnalogSensor = null;
        }

        if (Params.HAS_EXIT_SENSOR)
        {
            exitAnalogSensor = FtcOpMode.getInstance().hardwareMap.get(
                RevColorSensorV3.class, Params.EXIT_SENSOR_NAME);
            spindexerParams.setExitAnalogSourceTrigger(
                Params.EXIT_SENSOR_NAME, this::getExitSensorData, Params.EXIT_TRIGGER_LOW_THRESHOLD,
                Params.EXIT_TRIGGER_HIGH_THRESHOLD, Params.EXIT_TRIGGER_SETTLING, false, null, null);
        }
        else
        {
            exitAnalogSensor = null;
        }

        spindexer = new FtcPidStorage(Params.SUBSYSTEM_NAME, spindexerParams).getPidStorage();
        spindexer.motor.setPositionPidParameters(
            Params.posPidCoeffs, Params.POS_PID_TOLERANCE, Params.SOFTWARE_PID_ENABLED);
        entryPresets = new TrcPresets(
            Params.ENTRY_PRESETS_NAME, Params.POS_PRESET_TOLERANCE, Params.entryPresetPositions);
        exitPresets = new TrcPresets(
            Params.EXIT_PRESETS_NAME, Params.POS_PRESET_TOLERANCE, Params.exitPresetPositions);
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

    public boolean isEntrySensorActive()
    {
        return spindexer.isEntrySensorActive();
    }   //isEntrySensorActive

    public boolean isExitSensorActive()
    {
        return spindexer.isExitSensorActive();
    }   //isExitSensorActive

    public void setEntryPosition()
    {

    }   //setEntryPosition

    public void setExitPosition(Vision.ColorBlobType artifactType)
    {

    }   //setExitPosition

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
                lineNum++, "%s: power=%.3f, current=%.3f, pos=%.3f/%.3f, LimitSw=%s/%s",
                Params.SUBSYSTEM_NAME, spindexer.motor.getPower(), spindexer.motor.getCurrent(),
                spindexer.motor.getPosition(), spindexer.motor.getPidTarget(),
                spindexer.motor.isLowerLimitSwitchActive());
        }

        return lineNum;
    }   //updateStatus

    /**
     * This method is called to prep the subsystem for tuning.
     *
     * @param subComponent specifies the sub-component of the Subsystem to be tuned, can be null if no sub-component.
     * @param tuneParams specifies tuning parameters.
     *        tuneParam0 - Kp
     *        tuneParam1 - Ki
     *        tuneParam2 - Kd
     *        tuneParam3 - Kf
     *        tuneParam4 - iZone
     *        tuneParam5 - PidTolerance
     *        tuneParam6 - GravityCompPower
     */
    @Override
    public void prepSubsystemForTuning(String subComponent, double... tuneParams)
    {
        spindexer.motor.setPositionPidParameters(
            tuneParams[0], tuneParams[1], tuneParams[2], tuneParams[3], tuneParams[4], tuneParams[5],
            Params.SOFTWARE_PID_ENABLED);
    }   //prepSubsystemForTuning

}   //class Spindexer
