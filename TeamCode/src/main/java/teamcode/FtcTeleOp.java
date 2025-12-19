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

package teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Locale;

import ftclib.drivebase.FtcSwerveBase;
import ftclib.driverio.FtcGamepad;
import ftclib.robotcore.FtcOpMode;
import teamcode.indicators.LEDIndicator;
import teamcode.indicators.RumbleIndicator;
import teamcode.subsystems.Shooter;
import teamcode.vision.Vision;
import trclib.drivebase.TrcDriveBase;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcDbgTrace;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcRobot;
import trclib.subsystem.TrcShootParams;
import trclib.timer.TrcTimer;

/**
 * This class contains the TeleOp Mode program.
 */
@TeleOp(name="FtcTeleOp", group="Ftc3543")
public class FtcTeleOp extends FtcOpMode
{
    private final String moduleName = getClass().getSimpleName();

    protected Robot robot;
    protected FtcGamepad driverGamepad;
    protected FtcGamepad operatorGamepad;
    protected RumbleIndicator driverRumble;
    protected RumbleIndicator operatorRumble;
    private double drivePowerScale;
    private double turnPowerScale;
    protected boolean driverAltFunc = false;
    protected boolean operatorAltFunc = false;
    protected boolean allowAnalogControl = true;
    private boolean relocalizing = false;
    private int relocalizeCount = 0;
    private Integer savedLimelightPipeline = null;

    private double panPrevPower = 0.0;
    private double tiltPrevPower = 0.0;

    //
    // Implements FtcOpMode abstract method.
    //

    /**
     * This method is called to initialize the robot. In FTC, this is called when the "Init" button on the Driver
     * Station is pressed.
     */
    @Override
    public void robotInit()
    {
        // Create and initialize robot object.
        robot = new Robot(TrcRobot.getRunMode());
        // Open trace log.
        if (RobotParams.Preferences.useTraceLog)
        {
            String filePrefix = Robot.matchInfo != null?
                String.format(Locale.US, "%s%02d_TeleOp", Robot.matchInfo.matchType, Robot.matchInfo.matchNumber):
                "Standalone_TeleOp";
            TrcDbgTrace.openTraceLog(RobotParams.Robot.LOG_FOLDER_PATH, filePrefix);
        }
        // Create and initialize Gamepads.
        driverGamepad = new FtcGamepad("DriverGamepad", gamepad1);
        driverGamepad.setButtonEventHandler(this::driverButtonEvent);
        driverGamepad.setLeftStickInverted(false, true);
        driverGamepad.setRightStickInverted(false, true);

        operatorGamepad = new FtcGamepad("OperatorGamepad", gamepad2);
        operatorGamepad.setButtonEventHandler(this::operatorButtonEvent);
        operatorGamepad.setLeftStickInverted(false, true);
        operatorGamepad.setRightStickInverted(false, true);

        if (RobotParams.Preferences.useRumble)
        {
            driverRumble = new RumbleIndicator("DriverRumble", driverGamepad);
            operatorRumble = new RumbleIndicator("OperatorRumble", operatorGamepad);
        }

        drivePowerScale = Dashboard.Subsystem_Drivebase.driveNormalScale;
        turnPowerScale = Dashboard.Subsystem_Drivebase.turnNormalScale;
        setDriveOrientation(Dashboard.Subsystem_Drivebase.driveOrientation);
    }   //robotInit

    //
    // Overrides TrcRobot.RobotMode methods.
    //

    /**
     * This method is called when the competition mode is about to start. In FTC, this is called when the "Play"
     * button on the Driver Station is pressed. Typically, you put code that will prepare the robot for start of
     * competition here such as resetting the encoders/sensors and enabling some sensors to start sampling.
     *
     * @param prevMode specifies the previous RunMode it is coming from (always null for FTC).
     * @param nextMode specifies the next RunMode it is going into.
     */
    @Override
    public void startMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
        if (TrcDbgTrace.isTraceLogOpened())
        {
            TrcDbgTrace.setTraceLogEnabled(true);
        }
        robot.globalTracer.traceInfo(
            moduleName, "***** Starting TeleOp: " + TrcTimer.getCurrentTimeString() + " *****");
        robot.dashboard.clearDisplay();
        //
        // Tell robot object opmode is about to start so it can do the necessary start initialization for the mode.
        //
        robot.startMode(nextMode);
        //
        // Enable AprilTag vision for re-localization and AutoShoot.
        //
        if (robot.vision != null)
        {
            if (robot.vision.webcamAprilTagVision != null)
            {
                robot.globalTracer.traceInfo(moduleName, "Enabling WebCam AprilTagVision.");
                robot.vision.setWebcamAprilTagVisionEnabled(true);
            }
            else if (robot.vision.limelightVision != null)
            {
                robot.globalTracer.traceInfo(moduleName, "Enabling Limelight AprilTagVision.");
                robot.vision.setLimelightVisionEnabled(Vision.LimelightPipelineType.APRIL_TAG, true);
            }
        }
    }   //startMode

    /**
     * This method is called when competition mode is about to end. Typically, you put code that will do clean
     * up here such as disabling the sampling of some sensors.
     *
     * @param prevMode specifies the previous RunMode it is coming from.
     * @param nextMode specifies the next RunMode it is going into (always null for FTC).
     */
    @Override
    public void stopMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
        //
        // Tell robot object opmode is about to stop so it can do the necessary cleanup for the mode.
        //
        robot.stopMode(prevMode);
        robot.globalTracer.traceInfo(
            moduleName, "***** Stopping TeleOp: " + TrcTimer.getCurrentTimeString() + " *****");
        printPerformanceMetrics();

        if (TrcDbgTrace.isTraceLogOpened())
        {
            TrcDbgTrace.closeTraceLog();
        }
    }   //stopMode

    /**
     * This method is called periodically on the main robot thread. Typically, you put TeleOp control code here that
     * doesn't require frequent update For example, TeleOp joystick code or status display code can be put here since
     * human responses are considered slow.
     *
     * @param elapsedTime specifies the elapsed time since the mode started.
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false otherwise.
     */
    @Override
    public void periodic(double elapsedTime, boolean slowPeriodicLoop)
    {
        robot.periodic(elapsedTime, slowPeriodicLoop);
        if (allowAnalogControl)
        {
            if (slowPeriodicLoop)
            {
                //
                // DriveBase subsystem.
                //
                if (robot.robotBase != null)
                {
                    // We are trying to re-localize the robot and vision hasn't seen AprilTag yet.
                    if (relocalizing)
                    {
                        // Relocalize twice, using MT1 for the first time and MT2 for the second time.
                        if (relocalizeCount < 2)
                        {
                            TrcPose2D robotFieldPose = robot.vision.getRobotFieldPose();
                            if (robotFieldPose != null)
                            {
                                robot.relocalizedRobotPose =
                                    robot.shooterSubsystem.adjustRobotFieldPosition(robotFieldPose);
                                // Vision found an AprilTag, set the new robot field location.
                                robot.globalTracer.traceInfo(
                                    moduleName,
                                    ">>>>> Relocalizing: pose=" + robotFieldPose +
                                    ", adjPose=" + robot.relocalizedRobotPose);
                                robot.robotBase.driveBase.setFieldPosition(robot.relocalizedRobotPose, false);
                                Dashboard.DashboardParams.alliance =
                                    robot.vision.lastFieldAprilTagId == 24 ?
                                        FtcAuto.Alliance.RED_ALLIANCE : FtcAuto.Alliance.BLUE_ALLIANCE;
                                relocalizeCount++;
                            }
                        }
                    }
                    else
                    {
                        double[] inputs = driverGamepad.getDriveInputs(
                            Dashboard.Subsystem_Drivebase.driveMode, true, drivePowerScale, turnPowerScale);

                        if (robot.robotBase.driveBase.supportsHolonomicDrive())
                        {
                            robot.robotBase.driveBase.holonomicDrive(
                                null, inputs[0], inputs[1], inputs[2], robot.robotBase.driveBase.getDriveGyroAngle());
                        }
                        else
                        {
                            robot.robotBase.driveBase.arcadeDrive(inputs[1], inputs[2]);
                        }

                        if (robot.dashboard.isDashboardUpdateEnabled() && RobotParams.Preferences.showDriveBaseStatus)
                        {
                            robot.dashboard.displayPrintf(
                                14, "RobotDrive: Power=(%.2f,y=%.2f,rot=%.2f),Mode:%s",
                                inputs[0], inputs[1], inputs[2], robot.robotBase.driveBase.getDriveOrientation());
                        }
                    }
                    // Check for EndGame warning.
                    if (elapsedTime > RobotParams.Game.ENDGAME_DEADLINE)
                    {
                        if (driverRumble != null)
                        {
                            driverRumble.setRumblePattern(RumbleIndicator.ENDGAME_DEADLINE);
                        }

                        if (operatorRumble != null)
                        {
                            operatorRumble.setRumblePattern(RumbleIndicator.ENDGAME_DEADLINE);
                        }
                    }
                }
                //
                // Other subsystems.
                //
                if (RobotParams.Preferences.useSubsystems)
                {
                    // Analog control of subsystems.
                    if (robot.shooterSubsystem != null)
                    {
                        double panPower = operatorGamepad.getLeftStickX(true);
                        double tiltPower = operatorGamepad.getRightStickY(true);

                        if (panPower != panPrevPower && !robot.shooterSubsystem.isGoalTrackingEnabled())
                        {
                            if (operatorAltFunc)
                            {
                                robot.shooter.panMotor.setPower(panPower);
                            }
                            else
                            {
                                robot.shooter.panMotor.setPidPower(
                                    panPower, Shooter.Params.PAN_MIN_POS, Shooter.Params.PAN_MAX_POS, true);
                            }
                            panPrevPower = panPower;
                        }

                        if (tiltPower != tiltPrevPower)
                        {
                            if (operatorAltFunc)
                            {
                                robot.shooter.tiltMotor.setPower(tiltPower);
                            }
                            else
                            {
                                robot.shooter.tiltMotor.setPidPower(
                                    tiltPower, Shooter.Params.TILT_MIN_POS, Shooter.Params.TILT_MAX_POS, true);
                            }
                            tiltPrevPower = tiltPower;
                        }
                    }
                }
            }
        }
    }   //periodic

    /**
     * This method sets the drive orientation mode and updates the LED to indicate so.
     *
     * @param orientation specifies the drive orientation (FIELD, ROBOT, INVERTED).
     */
    private void setDriveOrientation(TrcDriveBase.DriveOrientation orientation)
    {
        if (robot.robotBase != null)
        {
            robot.globalTracer.traceInfo(moduleName, "driveOrientation=" + orientation);
            robot.robotBase.driveBase.setDriveOrientation(orientation, false);
            if (orientation == TrcDriveBase.DriveOrientation.FIELD)
            {
                robot.robotBase.driveBase.setFieldForwardHeading(
                    Dashboard.DashboardParams.alliance == FtcAuto.Alliance.RED_ALLIANCE? 0.0: 180.0);
            }
            if (robot.ledIndicator != null)
            {
                robot.ledIndicator.setDriveOrientation(orientation);
            }
        }
    }   //setDriveOrientation

    //
    // Implements TrcGameController.ButtonHandler interface.
    //

    /**
     * This method is called when driver gamepad button event is detected.
     *
     * @param button specifies the button that generates the event.
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    public void driverButtonEvent(FtcGamepad.ButtonType button, boolean pressed)
    {
        robot.dashboard.displayPrintf(15, "Driver: %s=%s", button, pressed? "Pressed": "Released");

        switch (button)
        {
            case A:
                setIntakeMode(pressed, driverAltFunc);
                break;

            case B:
                if (pressed)
                {
                    shootArtifacts(driverAltFunc);
                }
                break;

            case X:
                if (pressed)
                {
                    setDriveMode(driverAltFunc);
                }
                break;

            case Y:
                if (pressed)
                {
                    setGoalTrackingMode(driverAltFunc);
                }
                break;

            case LeftBumper:
                robot.globalTracer.traceInfo(moduleName, ">>>>> DriverAltFunc=" + pressed);
                driverAltFunc = pressed;
                break;

            case RightBumper:
                setDriveSpeedMode(pressed, driverAltFunc);
                break;

            case DpadUp:
            case DpadDown:
            case DpadLeft:
            case DpadRight:
                break;

            case Back:
                if (pressed)
                {
                    if (!driverAltFunc)
                    {
                        zeroCalibrate();
                    }
                    else
                    {
                        resetSwerveSteering();
                    }
                }
                break;

            case Start:
                relocalize(pressed);
                break;
        }
    }   //driverButtonEvent

    /**
     * This method is called when operator gamepad button event is detected.
     *
     * @param button specifies the button that generates the event.
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    public void operatorButtonEvent(FtcGamepad.ButtonType button, boolean pressed)
    {
        robot.dashboard.displayPrintf(15, "Operator: %s=%s", button, pressed? "Pressed": "Released");

        switch (button)
        {
            case A:
                setIntakeMode(pressed, operatorAltFunc);
                break;

            case B:
                if (pressed)
                {
                    shootArtifacts(operatorAltFunc);
                }
                break;

            case X:
                shooterLaunch(pressed, operatorAltFunc);
                break;

            case Y:
                if (pressed)
                {
                    refreshSpindexerSlots();
                }
                break;

            case LeftBumper:
                robot.globalTracer.traceInfo(moduleName, ">>>>> OperatorAltFunc=" + pressed);
                operatorAltFunc = pressed;
                break;

            case RightBumper:
                if (pressed && operatorAltFunc)
                {
                    toggleDashboardUpdateMode();
                }
                break;

            case DpadUp:
                if (pressed)
                {
                    robot.globalTracer.traceInfo(moduleName, ">>>>> Setting numArtifactsToShoot to 3");
                    Dashboard.Subsystem_Shooter.autoShootParams.numArtifactsToShoot = 3;
                }
                break;

            case DpadDown:
                if (pressed)
                {
                    robot.globalTracer.traceInfo(moduleName, ">>>>> Setting numArtifactsToShoot to 1");
                    Dashboard.Subsystem_Shooter.autoShootParams.numArtifactsToShoot = 1;
                }
                break;

            case DpadLeft:
                if (pressed)
                {
                    spindexerBackward(operatorAltFunc);
                }
                break;

            case DpadRight:
                if (pressed)
                {
                    spindexerForward(operatorAltFunc);
                }
                break;

            case Back:
                if (pressed)
                {
                    zeroCalibrate();
                }
                break;

            case Start:
                if (operatorAltFunc && pressed)
                {
                    Dashboard.DashboardParams.alliance =
                        Dashboard.DashboardParams.alliance == FtcAuto.Alliance.BLUE_ALLIANCE?
                            FtcAuto.Alliance.RED_ALLIANCE: FtcAuto.Alliance.BLUE_ALLIANCE;
                }
                break;
        }
    }   //operatorButtonEvent

    /**
     * This method is called to set Intake mode.
     *
     * @param pressed specifies true if the button is pressed, false if released.
     * @param altFunc specifies true if AltFunc is pressed, false otherwise.
     */
    private void setIntakeMode(boolean pressed, boolean altFunc)
    {
        if (robot.intakeSubsystem != null && robot.spindexerSubsystem != null)
        {
            if (!altFunc)
            {
                if (pressed)
                {
                    if (robot.intake.isActive())
                    {
                        robot.globalTracer.traceInfo(moduleName, ">>>>> Cancel Bulldoze Intake");
                        robot.intakeSubsystem.setBulldozeIntakeEnabled(false, null, null);
                    }
                    else
                    {
                        robot.globalTracer.traceInfo(moduleName, ">>>>> Bulldoze Intake");
                        robot.intakeSubsystem.setBulldozeIntakeEnabled(true, null, null);
                    }
                }
            }
            else
            {
                // Cancel Bulldoze Intake in case it's enabled.
                robot.intakeSubsystem.setBulldozeIntakeEnabled(false, null, null);
                if (pressed)
                {
                    robot.globalTracer.traceInfo(moduleName, ">>>>> Start Intake Eject");
                    robot.intake.eject();
                }
                else
                {
                    robot.globalTracer.traceInfo(moduleName, ">>>>> Cancel Intake Eject");
                    robot.intake.cancel();
                }
            }
        }
    }   //setIntakeMode

    /**
     * This method is called to shoot the artifacts.
     *
     * @param altFunc specifies true if AltFunc is pressed, false otherwise.
     */
    private void shootArtifacts(boolean altFunc)
    {
        if (robot.shooterSubsystem != null && robot.spindexerSubsystem != null)
        {
            if (!altFunc && robot.autoShootTask != null)
            {
                // Auto Shoot Task is enabled, auto shoot at any AprilTag detected.
                if (robot.autoShootTask.isActive())
                {
                    robot.globalTracer.traceInfo(moduleName, ">>>>> Cancel Auto Shoot");
                    robot.autoShootTask.cancel();
                }
                else
                {
                    robot.globalTracer.traceInfo(moduleName, ">>>>> Auto Shoot");
                    robot.autoShootTask.autoShoot(
                        moduleName + ".autoShoot", null,
                        Dashboard.DashboardParams.alliance,
                        false,
                        Dashboard.Subsystem_Shooter.autoShootParams.useAprilTagVision,
                        Dashboard.Subsystem_Shooter.autoShootParams.doMotif,
                        Dashboard.Subsystem_Shooter.autoShootParams.useClassifierVision,
                        Dashboard.Subsystem_Shooter.autoShootParams.useRegression,
                        Dashboard.Subsystem_Shooter.autoShootParams.flywheelTracking,
                        Dashboard.Subsystem_Shooter.autoShootParams.relocalize,
                        Dashboard.Subsystem_Shooter.autoShootParams.numArtifactsToShoot > 0 ?
                            Dashboard.Subsystem_Shooter.autoShootParams.numArtifactsToShoot : 1,
                        Dashboard.Subsystem_Shooter.autoShootParams.moveToNextExitSlot);
                }
            }
            else
            {
                // Manual shoot is used when sensors are not functioning properly. Therefore, this mode
                // doesn't depend on Vision nor Spindexer color sensor. It is assuming either we have auto
                // Goal Tracking by Odometry turned ON that does the aiming or the driver has to drive the
                // robot to a fixed location (FAR_ZONE_SHOOT_POINT) and it will shoot with preset parameters
                // at that location.
                robot.globalTracer.traceInfo(moduleName, ">>>>> Manual Shoot");
                // Cancel AutoShoot in case it's active.
                if (robot.autoShootTask != null)
                {
                    robot.autoShootTask.cancel();
                }

                if (robot.shooterSubsystem.isGoalTrackingEnabled())
                {
                    TrcEvent callbackEvent = new TrcEvent(moduleName + ".manualShoot");
                    callbackEvent.setCallback(
                        (ctxt, canceled) ->
                        {
                            if (!canceled)
                            {
                                robot.shooterSubsystem.shoot(null, null);
                            }
                        },
                        null);
                    if (!robot.spindexerSubsystem.moveToExitSlotWithArtifact(
                        null, Vision.ArtifactType.Any, callbackEvent))
                    {
                        callbackEvent.clear();
                        // Spindexer is empty, try shooting anyway.
                        robot.spindexerSubsystem.exitSlotUp(null, callbackEvent);
                    }
                }
                else
                {
                    // Not using any auto aiming, fixed point shooting at FAR_ZONE. Get ShootParams
                    // at the fixed point, set flywheel speed and tilt accordingly and just shoot.
                    // Drive is responsible for driving to the FAR_ZONE_SHOOT_POINT and control the
                    // turret to aim at the goal, the code will do the rest.
                    TrcShootParams.Entry manualShootParams = Shooter.shootParamsTable.get(Shooter.FAR_ZONE_SHOOT_POINT);
                    // Fire and forget assuming Spindexer moves faster than aimShooter.
                    if (!robot.spindexerSubsystem.moveToExitSlotWithArtifact(
                        null, Vision.ArtifactType.Any, null))
                    {
                        // Spindexer is empty, try shooting anyway.
                        robot.spindexerSubsystem.exitSlotUp(null, null);
                    }
                    // Note: since we are doing fire and forget on the tilt angle, we assume tilt will
                    // get on target before the flywheel.
                    robot.shooter.setTiltAngle(moduleName, manualShootParams.region.tiltAngle, null, 0.0);
                    robot.shooter.aimShooter(
                        moduleName, manualShootParams.outputs[0]/60.0, 0.0, null, null, null, 0.0,
                        robot.shooterSubsystem::shoot, Shooter.Params.SHOOT_MOTOR_OFF_DELAY);
                }
            }
        }
    }   //shootArtifacts

    /**
     * This method is called to change the drive mode between ROBOT mode and FIELD mode or turning ON/OFF GyroAssist.
     *
     * @param altFunc specifies true if AltFunc is pressed, false otherwise.
     */
    private void setDriveMode(boolean altFunc)
    {
        if (robot.robotBase != null)
        {
            if (altFunc)
            {
                if (robot.robotBase.driveBase.isGyroAssistEnabled())
                {
                    robot.globalTracer.traceInfo(moduleName, ">>>>> Disabling GyroAssist.");
                    robot.robotBase.driveBase.setGyroAssistEnabled(null);
                }
                else
                {
                    robot.globalTracer.traceInfo(moduleName, ">>>>> Enabling GyroAssist.");
                    robot.robotBase.driveBase.setGyroAssistEnabled(
                        robot.robotBase.purePursuitDrive.getTurnPidCtrl());
                }
            }
            else if (robot.robotBase.driveBase.supportsHolonomicDrive())
            {
                // Toggle between field or robot oriented driving, only applicable for holonomic drive base.
                if (robot.robotBase.driveBase.getDriveOrientation() != TrcDriveBase.DriveOrientation.FIELD)
                {
                    robot.globalTracer.traceInfo(moduleName, ">>>>> Enabling FIELD mode.");
                    setDriveOrientation(TrcDriveBase.DriveOrientation.FIELD);
                }
                else
                {
                    robot.globalTracer.traceInfo(moduleName, ">>>>> Enabling ROBOT mode.");
                    setDriveOrientation(TrcDriveBase.DriveOrientation.ROBOT);
                }
            }
        }
    }   //setDriveMode

    /**
     * This method is called to set various Goal Tracking modes.
     *
     * @param altFunc specifies true if AltFunc is pressed, false otherwise.
     */
    private void setGoalTrackingMode(boolean altFunc)
    {
        if (robot.shooterSubsystem != null)
        {
            if (robot.shooterSubsystem.isGoalTrackingEnabled())
            {
                robot.globalTracer.traceInfo(moduleName, ">>>>> Disable GoalTracking.");
                robot.shooterSubsystem.disableGoalTracking(null);
            }
            else if (!altFunc && robot.vision != null)
            {
                robot.shooterSubsystem.enableGoalTracking(null, true, Dashboard.DashboardParams.alliance, true);
                robot.globalTracer.traceInfo(
                    moduleName, ">>>>> Enable GoalTracking by AprilTag (alliance=%s).",
                    FtcAuto.autoChoices.alliance);
                if (robot.ledIndicator != null)
                {
                    robot.ledIndicator.setStatusPatternOn(
                        FtcAuto.autoChoices.alliance == FtcAuto.Alliance.BLUE_ALLIANCE?
                            LEDIndicator.BLUE_APRILTAG: LEDIndicator.RED_APRILTAG,
                        true);
                }
            }
            else
            {
                robot.shooterSubsystem.enableGoalTracking(null, false, Dashboard.DashboardParams.alliance, true);
                robot.globalTracer.traceInfo(
                    moduleName, ">>>>> Enable GoalTracking by Odometry (alliance=%s).",
                    FtcAuto.autoChoices.alliance);
                if (robot.ledIndicator != null)
                {
                    robot.ledIndicator.setStatusPatternOn(
                        FtcAuto.autoChoices.alliance == FtcAuto.Alliance.BLUE_ALLIANCE?
                            LEDIndicator.BLUE_GOAL: LEDIndicator.RED_GOAL,
                        true);
                }
            }
        }
    }   //setGoalTrackingMode

    /**
     * This method is called to set drive speed modes.
     *
     * @param pressed specifies true if the button is pressed, false if released.
     * @param altFunc specifies true if AltFunc is pressed, false otherwise.
     */
    private void setDriveSpeedMode(boolean pressed, boolean altFunc)
    {
        if (!altFunc)
        {
            // Press and hold for slow drive.
            if (pressed)
            {
                robot.globalTracer.traceInfo(moduleName, ">>>>> DrivePower slow.");
                drivePowerScale = Dashboard.Subsystem_Drivebase.driveSlowScale;
                turnPowerScale = Dashboard.Subsystem_Drivebase.turnSlowScale;
            }
            else
            {
                robot.globalTracer.traceInfo(moduleName, ">>>>> DrivePower normal.");
                drivePowerScale = Dashboard.Subsystem_Drivebase.driveNormalScale;
                turnPowerScale = Dashboard.Subsystem_Drivebase.turnNormalScale;
            }
        }
        else
        {
            if (pressed)
            {
                toggleDashboardUpdateMode();
            }
        }
    }   //setDriveSpeedMode

    /**
     * This method is called to relocalize the robot.
     *
     * @param pressed specifies true if the button is pressed, false if released.
     */
    private void relocalize(boolean pressed)
    {
        // Do AprilTag Vision re-localization.
        if (robot.vision != null && robot.robotBase != null && robot.shooterSubsystem != null)
        {
            boolean hasAprilTagVision = robot.vision.isWebcamAprilTagVisionEnabled();
            // If Webcam AprilTag vision is not enabled, check if we have Limelight since Limelight has
            // AprilTag pipeline as well.
            if (!hasAprilTagVision && robot.vision.limelightVision != null)
            {
                hasAprilTagVision = true;
                if (pressed)
                {
                    // Webcam AprilTag vision is not enable, enable Limelight AprilTag pipeline instead.
                    // Note: we assume pipeline 0 is the AprilTag pipeline.
                    savedLimelightPipeline = robot.vision.limelightVision.getPipeline();
                    robot.vision.setLimelightVisionEnabled(Vision.LimelightPipelineType.APRIL_TAG, true);
                }
            }

            if (hasAprilTagVision)
            {
                relocalizing = pressed;
                // On press of the button, we will start looking for AprilTag for re-localization.
                // On release of the button, we will set the robot's field location if we found the AprilTag.
                if (pressed)
                {
                    relocalizeCount = 0;
                    robot.globalTracer.traceInfo(moduleName, ">>>>> Start re-localizing ...");
                }
                else if (savedLimelightPipeline != null)
                {
                    // Done with AprilTag re-localization, restore previous Limelight pipeline.
                    robot.vision.limelightVision.setPipeline(savedLimelightPipeline);
                    savedLimelightPipeline = null;
                }
            }
        }
    }   //relocalize

    /**
     * This method is called to zero calibrate all subsystems.
     */
    private void zeroCalibrate()
    {
        // Cancel all operations and zero calibrate all subsystems (arm, elevator and turret).
        robot.globalTracer.traceInfo(moduleName, ">>>>> ZeroCalibrating.");
        robot.cancelAll();
        robot.zeroCalibrate(null, null);
    }   //zeroCalibrate

    /**
     * This method is called to set all swerve steering to zero angle.
     */
    private void resetSwerveSteering()
    {
        // If drive base is SwerveDrive, set all wheels pointing forward.
        if (robot.robotBase != null && robot.robotBase instanceof FtcSwerveBase)
        {
            // Drive base is a Swerve Drive, align all steering wheels forward.
            robot.globalTracer.traceInfo(moduleName, ">>>>> Set SteerAngle to zero.");
            ((FtcSwerveBase) robot.robotBase).setSteerAngle(0.0, false, false);
        }
    }   //resetSwerveSteering

    /**
     * This method is called to launch an artifact in the Spindexer exit slot.
     *
     * @param pressed specifies true if the button is pressed, false if released.
     * @param altFunc specifies true if AltFunc is pressed, false otherwise.
     */
    private void shooterLaunch(boolean pressed, boolean altFunc)
    {
        if (robot.shooterSubsystem != null)
        {
            // Cancel AutoShoot in case it's active.
            if (robot.autoShootTask != null)
            {
                robot.autoShootTask.cancel();
            }

            if (robot.spindexerSubsystem != null && robot.spindexerSubsystem.getExitSlotArtifactType() == null)
            {
                // Exit slot is not aligned. Align it first.
                // Fire and forget assuming Spindex can get out of the way fast enough.
                robot.spindexerSubsystem.exitSlotUp(null, null);
            }

            if (altFunc)
            {
                robot.globalTracer.traceInfo(moduleName, ">>>>> setLaunchPosition=" + pressed);
                robot.shooterSubsystem.setLauncherPosition(null, pressed);
            }
            else if (pressed)
            {
                robot.globalTracer.traceInfo(moduleName, ">>>>> Shoot");
                robot.shooterSubsystem.shoot(null, null);
            }
        }
    }   //shooterLaunch

    /**
     * This method is called to refresh Spindexer slot states to determine what artifact is in each slot.
     */
    private void refreshSpindexerSlots()
    {
        if (robot.spindexerSubsystem != null && robot.intakeSubsystem != null)
        {
            robot.globalTracer.traceInfo(moduleName, ">>>>> refreshSlotStates");
            robot.spindexerSubsystem.refreshSlotStates();
        }
    }   //refreshSpindexerSlots

    /**
     * This method is called to toggle DashboardUpdate mode.
     */
    private void toggleDashboardUpdateMode()
    {
        boolean enabled = !robot.dashboard.isDashboardUpdateEnabled();
        robot.globalTracer.traceInfo(moduleName, ">>>>> setUpdateDashboardEnable=" + enabled);
        if (enabled)
        {
            robot.dashboard.enableDashboardUpdate(1, true);
        }
        else
        {
            robot.dashboard.disableDashboardUpdate();
        }
    }   //toggleDashboardUpdateMode

    /**
     * This method is called to spin the Spindexer to one slot backward.
     *
     * @param altFunc specifies true if AltFunc is pressed, false otherwise.
     */
    private void spindexerBackward(boolean altFunc)
    {
        if (robot.spindexer != null)
        {
            if (altFunc)
            {
                robot.globalTracer.traceInfo(moduleName, ">>>>> Backup Spindexer exit position.");
                robot.spindexerSubsystem.exitSlotDown(null, null);
            }
            else
            {
                robot.globalTracer.traceInfo(moduleName, ">>>>> Backup Spindexer entry position.");
                robot.spindexerSubsystem.entrySlotDown(null, null);
            }
        }
    }   //spindexerBackward

    /**
     * This method is called to spin the Spindexer to one slot forward.
     *
     * @param altFunc specifies true if AltFunc is pressed, false otherwise.
     */
    private void spindexerForward(boolean altFunc)
    {
        if (altFunc)
        {
            robot.globalTracer.traceInfo(moduleName, ">>>>> Advance Spindexer exit position.");
            robot.spindexerSubsystem.exitSlotUp(null, null);
        }
        else
        {
            robot.globalTracer.traceInfo(moduleName, ">>>>> Advance Spindexer entry position.");
            robot.spindexerSubsystem.entrySlotUp(null, null);
        }
    }   //spindexerForward

}   //class FtcTeleOp
