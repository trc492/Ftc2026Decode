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

import teamcode.Robot;
import ftclib.driverio.FtcDashboard;
import ftclib.motor.FtcMotorActuator.MotorType;
import ftclib.subsystem.FtcRollerIntake;
import teamcode.RobotParams;
import teamcode.indicators.LEDIndicator;
import teamcode.vision.Vision;
import trclib.robotcore.TrcEvent;
import trclib.sensor.TrcTrigger;
import trclib.subsystem.TrcRollerIntake;
import trclib.subsystem.TrcSubsystem;
import trclib.subsystem.TrcRollerIntake.TriggerAction;
import trclib.timer.TrcTimer;
import trclib.vision.TrcOpenCvColorBlobPipeline;
import trclib.vision.TrcVisionTargetInfo;

/**
 * This class implements an Intake Subsystem. This implementation consists of one motor but no sensor of its own.
 * It does have two triggers, one front and one back. The front trigger is triggered by vision detecting if there is
 * a correct color artifact in front of it. The back trigger is triggered by the entry sensor of the spindexer so
 * it can stop the Intake.
 */
public class Intake extends TrcSubsystem
{
    public static final class Params
    {
        public static final String SUBSYSTEM_NAME               = "Intake";
        public static final boolean NEED_ZERO_CAL               = false;

        public static final boolean HAS_FRONT_TRIGGER           = true;
        public static final boolean HAS_BACK_TRIGGER            = true;

        public static final String MOTOR_NAME                   = SUBSYSTEM_NAME + ".Motor";
        public static final MotorType MOTOR_TYPE                = MotorType.DcMotor;
        public static final boolean MOTOR_INVERTED              =
            RobotParams.Preferences.robotType == DriveBase.RobotType.SwerveRobot;

        public static final String FRONT_TRIGGER_NAME           = SUBSYSTEM_NAME + ".FrontTrigger";
        public static final String BACK_TRIGGER_NAME            = SUBSYSTEM_NAME + ".BackTrigger";

        public static final double INTAKE_POWER                 = 1.0;  // Intake forward
        public static final double EJECT_POWER                  = -1.0;
        public static final double RETAIN_POWER                 = 0.0;
        public static final double INTAKE_FINISH_DELAY          = 0.5;
        public static final double EJECT_FINISH_DELAY           = 0.5;
    }   //class Params

    private final FtcDashboard dashboard;
    private final Robot robot;
    private final TrcRollerIntake intake;
    private final TrcTimer timer;
    private Vision.ArtifactType pickupArtifactType = Vision.ArtifactType.Any;
    private String detectedArtifactName = null;
    private boolean bulldozeEnabled = false;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param robot specifies the robot object to access the other subsystems.
     */
    public Intake(Robot robot)
    {
        super(Params.SUBSYSTEM_NAME, Params.NEED_ZERO_CAL);

        this.dashboard = FtcDashboard.getInstance();
        this.robot = robot;
        FtcRollerIntake.Params intakeParams = new FtcRollerIntake.Params()
            .setPrimaryMotor(Params.MOTOR_NAME, Params.MOTOR_TYPE, Params.MOTOR_INVERTED)
            .setPowerLevels(Params.INTAKE_POWER, Params.EJECT_POWER, Params.RETAIN_POWER)
            .setFinishDelays(Params.INTAKE_FINISH_DELAY, Params.EJECT_FINISH_DELAY);

        if (Params.HAS_FRONT_TRIGGER)
        {
            intakeParams.setFrontDigitalSourceTrigger(
                Params.FRONT_TRIGGER_NAME, this::visionDetectedArtifact, TriggerAction.StartOnTrigger,
                TrcTrigger.TriggerMode.OnActive, this::frontTriggerCallback, null);
        }

        if (Params.HAS_BACK_TRIGGER)
        {
            intakeParams.setBackDigitalSourceTrigger(
                Params.BACK_TRIGGER_NAME, this::spindexerEntryHasArtifact, TriggerAction.FinishOnTrigger,
                null, null, null);
        }
        intake = new FtcRollerIntake(Params.SUBSYSTEM_NAME, intakeParams).getIntake();
        timer = new TrcTimer(Params.SUBSYSTEM_NAME + ".timer");
    }   //Intake

    /**
     * This method returns the created TrcRollerIntake.
     *
     * @return created Roller Intake.
     */
    public TrcRollerIntake getIntake()
    {
        return intake;
    }   //getIntake

    /**
     * This method is called by the Intake front trigger periodically using vision to detect the correct artifact type
     * to be picked up. Spindexer is responsible for calling setPickupArtifactType to specify whether vision should
     * look for purple artifact, green artifact, any artifact or None.
     *
     * @return true if vision found the specified artifact type, false otherwise.
     */
    private boolean visionDetectedArtifact()
    {
        boolean artifactDetected = false;

        if (robot.vision != null && robot.vision.artifactVision != null &&
            pickupArtifactType != Vision.ArtifactType.None)
        {
            TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject> artifactInfo =
                robot.vision.artifactVision.getBestDetectedTargetInfo(
                    robot.vision::artifactFilter, pickupArtifactType, robot.vision::compareDistanceY, 0.0,
                    robot.robotInfo.webCam1.camPose.z);
            artifactDetected = artifactInfo != null;
            detectedArtifactName = artifactDetected? artifactInfo.detectedObj.label: null;
        }

        return artifactDetected;
    }   //visionDetectedArtifact

    /**
     * This method is called when the front sensor is triggered.
     * While vision kept finding correct objects, it will keep calling this method and keep enabling the spindexer
     * entry trigger. That's okay because it does no harm enabling a trigger that's already enabled. It will just
     * be ignored.
     *
     * @param context (not used).
     * @param canceled specifies if the trigger is disabled.
     */
    private void frontTriggerCallback(Object context, boolean canceled)
    {
        // Vision found the artifact we want. Enable Spindexer entry trigger preparing to receive it.
        intake.tracer.traceInfo(instanceName, "Vision found artifact " + detectedArtifactName);
        if (!canceled && robot.spindexer != null)
        {
            robot.spindexer.setEntryTriggerEnabled(true);
        }
    }   //frontTriggerCallback

    /**
     * This method is called by the Intake back trigger periodically using the spindexer entry sensor to detect if the
     * artifact has entered the spindexer, so it can stop the Intake.
     *
     * @return true if spindexer entry sensor has detected the artifact, false otherwise.
     */
    private boolean spindexerEntryHasArtifact()
    {
        return robot.spindexer != null && robot.spindexer.isEntrySensorActive();
    }   //spindexerEntryHasArtifact

    /**
     * This method is called by the Spindexer to set the artifact type to pick up. This is according to what artifacts
     * are already in the Spindexer. It will ask for Any artifact if Spindexer is empty or has one Purple artifact.
     * It will ask for Purple if it has one or two vacant slots and already has a Green. It will ask for None if
     * Spindexer is full.
     *
     * @param artifactType specifies the artifact to pick up.
     */
    public void setPickupArtifactType(Vision.ArtifactType artifactType)
    {
        pickupArtifactType = artifactType;
        intake.tracer.traceInfo(instanceName, "Expect to pick up artifact " + pickupArtifactType);
    }   //setPickupArtifactType

    /**
     * This method enables/disable Bulldoze intake of artifacts. When enabled, it turns on manual intake and also
     * turns on Spindexer auto receive.
     *
     * @param enabled specifies true to enable and false to disable.
     * @param power specifies the intake power, if null use default power (only applicable for enabling).
     * @param event specifies an event to notify if spindexer is full (only applicable for enabling), null if not
     *        provided.
     */
    public void setBulldozeIntakeEnabled(boolean enabled, Double power, TrcEvent event)
    {
        boolean intakeOn = intake.isActive();

        if (!intakeOn && enabled)
        {
            // Enabling Bulldoze Intake, turn on manual intake and Spindexer AutoReceive.
            intake.tracer.traceInfo(instanceName, "Turning on Intake.");
            intake.intake(power);
            robot.spindexerSubsystem.setAutoReceiveEnabled(true, event);
            bulldozeEnabled = true;
        }
        else if (intakeOn && !enabled)
        {
            // Disabling Bulldoze Intake, turn off manual intake and Spindexer AutoReceive.
            intake.tracer.traceInfo(instanceName, "Turning off Intake.");
            timer.set(Params.INTAKE_FINISH_DELAY, (ctxt, canceled) -> {intake.cancel(); if (event != null) event.signal();});
            robot.spindexerSubsystem.setAutoReceiveEnabled(false, null);
            bulldozeEnabled = false;
        }

        if (robot.ledIndicator != null)
        {
            robot.ledIndicator.setStatusPatternState(LEDIndicator.INTAKE_ACTIVE, enabled);
        }
    }   //setBulldozeIntakeEnabled

    /**
     * This method checks if Bulldoze mode is enabled.
     *
     * @return true if bulldoze mode is enabled, false if disabled.
     */
    public boolean isBulldozeEnabled()
    {
        return bulldozeEnabled;
    }   //isBulldozeEnabled

    //
    // Implements TrcSubsystem abstract methods.
    //

    /**
     * This method cancels any pending operations.
     */
    @Override
    public void cancel()
    {
        intake.cancel();
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
        // Intake does not need zero calibration.
    }   //zeroCalibrate

    /**
     * This method resets the subsystem state. Typically, this is used to retract the subsystem for turtle mode.
     */
    @Override
    public void resetState()
    {
        // Intake does not support resetState.
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
        if (RobotParams.Preferences.showIntakeStatus)
        {
            if (slowLoop)
            {
                dashboard.displayPrintf(
                    lineNum++, "%s: power=%.1f, current=%.1f, front/back=%s/%s, auto=%s",
                    Params.SUBSYSTEM_NAME, intake.getPower(), intake.getCurrent(), intake.getFrontTriggerState(),
                    intake.getBackTriggerState(), intake.isAutoActive());
//                dashboard.displayPrintf(
//                    lineNum++, "%s: artifact(detected/expected)=%s/%s",
//                    Params.SUBSYSTEM_NAME, detectedArtifactName, pickupArtifactType);
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
        // Intake subsystem doesn't need tuning.
    }   //updateParamsToDashboard

    /**
     * This method is called to update subsystem parameters from the Dashboard.
     */
    @Override
    public void updateParamsFromDashboard()
    {
        // Intake subsystem doesn't need tuning.
    }   //updateParamsFromDashboard

}   //class Intake
