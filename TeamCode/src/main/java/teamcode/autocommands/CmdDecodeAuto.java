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

package teamcode.autocommands;

import teamcode.Dashboard;
import teamcode.FtcAuto;
import teamcode.Robot;
import teamcode.RobotParams;
import teamcode.subsystems.Shooter;
import teamcode.vision.Vision;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcStateMachine;
import trclib.subsystem.TrcShootParams;
import trclib.timer.TrcTimer;

/**
 * This class implements an autonomous strategy.
 */
public class CmdDecodeAuto implements TrcRobot.RobotCommand
{
    private static final String moduleName = CmdDecodeAuto.class.getSimpleName();
    private static final boolean useAutoGoalTracking = true;

    private enum State
    {
        START,
        GOTO_SHOOT_POS,
        SHOOT_ARTIFACTS,
        PICKUP_LOADING,
        MOVE_BACK,
        ATTEMPT_PICKUP,
        FINISH_LOADING,
        PICKUP_SPIKEMARK,
        FINISH_PICKUP,
        OPEN_GATE,
        PARK,
        DONE
    }   //enum State

    private final Robot robot;
    private final FtcAuto.AutoChoices autoChoices;
    private final TrcTimer timer;
    private final TrcEvent event;
    private final TrcEvent spindexerFullEvent;
    private final TrcStateMachine<State> sm;

    private int[] spikeMarkOrder = null;
    private int currentSpikeMarkCount = 0;
    private int targetSpikeMarkCount = 0;
    private boolean loadingPickedUp = false;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object for providing access to various global objects.
     * @param autoChoices specifies the autoChoices object.
     */
    public CmdDecodeAuto(Robot robot, FtcAuto.AutoChoices autoChoices)
    {
        this.robot = robot;
        this.autoChoices = autoChoices;

        timer = new TrcTimer(moduleName);
        event = new TrcEvent(moduleName);
        spindexerFullEvent = new TrcEvent(moduleName + ".spindexerFull");
        sm = new TrcStateMachine<>(moduleName);
        sm.start(State.START);
    }   //CmdDecodeAuto

    //
    // Implements the TrcRobot.RobotCommand interface.
    //

    /**
     * This method checks if the current RobotCommand  is running.
     *
     * @return true if the command is running, false otherwise.
     */
    @Override
    public boolean isActive()
    {
        return sm.isEnabled();
    }   //isActive

    /**
     * This method cancels the command if it is active.
     */
    @Override
    public void cancel()
    {
        timer.cancel();
        sm.stop();
        robot.robotBase.purePursuitDrive.setMoveOutputLimit(1.0);
        if (robot.intakeSubsystem != null)
        {
            robot.intakeSubsystem.setBulldozeIntakeEnabled(false, null, null);
        }
        if (robot.autoShootTask != null)
        {
            robot.autoShootTask.cancel();
        }
        if (robot.shooterSubsystem != null)
        {
            robot.shooterSubsystem.cancel();
        }
    }   //cancel

    /**
     * This method must be called periodically by the caller to drive the command sequence forward.
     *
     * @param elapsedTime specifies the elapsed time in seconds since the start of the robot mode.
     * @return true if the command sequence is completed, false otherwise.
     */
    @Override
    public boolean cmdPeriodic(double elapsedTime)
    {
        State state = sm.checkReadyAndGetState();

        if (state == null)
        {
            robot.dashboard.displayPrintf(8, "State: disabled or waiting (nextState=" + sm.getNextState() + ")...");
        }
        else
        {
            robot.dashboard.displayPrintf(8, "State: " + state);
            robot.globalTracer.tracePreStateInfo(sm.toString(), state);
            switch (state)
            {
                case START:
                    // Set robot location according to auto choices.
                    robot.setRobotStartPosition(autoChoices);
                    if (robot.spindexerSubsystem != null)
                    {
                        robot.spindexerSubsystem.setPreloadedArtifacts(
                            Vision.ArtifactType.Green, Vision.ArtifactType.Purple, Vision.ArtifactType.Purple);
                    }
                    spikeMarkOrder =
                        autoChoices.startPos == FtcAuto.StartPos.GOAL_ZONE? new int[] {0, 1, 2}:
                        autoChoices.openGate == FtcAuto.OpenGate.YES? new int[] {1, 0, 2}: new int[] {2, 1, 0};
                    targetSpikeMarkCount = (int) autoChoices.spikeMarkCount;
                    currentSpikeMarkCount = 0;

                    if (robot.shooterSubsystem != null)
                    {
                        TrcShootParams.Entry shootParams;
                        double panAngle;
                        // Turret was turned towards Obelisk before Auton is started, turn it back to the goal AprilTag.
                        if (autoChoices.startPos == FtcAuto.StartPos.GOAL_ZONE)
                        {
                            shootParams = Shooter.shootParamsTable.get(Shooter.GOAL_ZONE_SHOOT_POINT);
                            panAngle = autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE ? -45.0 : 45.0;
                        }
                        else
                        {
                            shootParams = Shooter.shootParamsTable.get(Shooter.FAR_ZONE_SHOOT_POINT);
                            panAngle = autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE ? -70.0 : 70.0;
                        }

                        if (useAutoGoalTracking)
                        {
                            TrcEvent callbackEvent = new TrcEvent(moduleName + ".goalTrackingCallbackEvent");
                            // Turn on AutoGoalTracking once the turret is facing the goal AprilTag.
                            callbackEvent.setCallback(
                                (ctxt, canceled) ->
                                {
                                    robot.globalTracer.traceInfo(
                                        moduleName, "GoalTrackingCallback(canceled=%s)", canceled);
                                    if (!canceled)
                                    {
                                        robot.globalTracer.traceInfo(moduleName, "enabling tracking");
                                        robot.shooterSubsystem.enableGoalTracking(
                                            null, true, autoChoices.alliance, true);
                                    }
                                }, null);
                            robot.shooter.setPanAngle(panAngle, callbackEvent, 0.0);
                            robot.globalTracer.traceInfo(
                                moduleName, "Set callback event to turn on auto tracking (event=%s)", callbackEvent);
                        }
                        // Pre-spin flywheel and set up pan/tilt angles for scoring artifacts (fire and forget).
                        robot.shooter.setTiltAngle(shootParams.region.tiltAngle);
                        robot.shooter.setShooterMotorRPM(shootParams.outputs[0], 0.0);
                        robot.globalTracer.traceInfo(
                            moduleName, "Pre-spin shooter(shootParams=" + shootParams + ")");
                    }
                    // Do delay if necessary.
                    if (autoChoices.startDelay > 0.0)
                    {
                        robot.globalTracer.traceInfo(
                            moduleName, "***** Do Start Delay " + autoChoices.startDelay + "s.");
                        timer.set(autoChoices.startDelay, event);
                        sm.waitForSingleEvent(event, State.GOTO_SHOOT_POS);
                    }
                    else
                    {
                        sm.setState(State.GOTO_SHOOT_POS);
                    }
                    break;

                case GOTO_SHOOT_POS:
                    if (autoChoices.startPos != FtcAuto.StartPos.GOAL_ZONE && currentSpikeMarkCount == 0 && !loadingPickedUp)
                    {
                        sm.setState(State.SHOOT_ARTIFACTS);
                    }
                    else
                    {
                        TrcPose2D shootPose;

                        if (autoChoices.startPos != FtcAuto.StartPos.GOAL_ZONE)
                        {
                            shootPose = RobotParams.Game.RED_FAR_ZONE_SHOOT_POSE;
                        }
                        else if (currentSpikeMarkCount == 0 ||
                                 autoChoices.classifierVision == FtcAuto.ClassifierVision.NO)
                        {
                            // Shoot in the GOAL_ZONE but not using ClassifierVision.
                            shootPose = RobotParams.Game.RED_GOAL_ZONE_SHOOT_POSE;
                        }
                        else
                        {
                            // Shoot in the GOAL_ZONE but with ClassifierVision.
                            // Need to back up a bit to see the entire classifier ramp.
                            shootPose = RobotParams.Game.RED_GOAL_ZONE_SHOOT_POSE.clone();
                            shootPose.x += RobotParams.Robot.ROBOT_WIDTH/2.0;
                            shootPose.y -= RobotParams.Robot.ROBOT_LENGTH/2.0;
                        }
                        robot.robotBase.purePursuitDrive.setMoveOutputLimit(1.0);
                        if (currentSpikeMarkCount == 2 && autoChoices.startPos == FtcAuto.StartPos.GOAL_ZONE)
                        {
                            TrcPose2D intermediatePose = robot.robotBase.driveBase.getFieldPosition();
                            intermediatePose.y += autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE ? -12.0 : 12.0;
                            robot.globalTracer.traceInfo(moduleName, "intermediatePose=%s", intermediatePose);
                            robot.robotBase.purePursuitDrive.start(
                                event, 0.0, false,
                                robot.robotInfo.baseParams.profiledMaxDriveVelocity,
                                robot.robotInfo.baseParams.profiledMaxDriveAcceleration,
                                robot.robotInfo.baseParams.profiledMaxDriveDeceleration,
                                intermediatePose,
                                robot.adjustPoseByAlliance(shootPose, autoChoices.alliance));
                        }
                        else
                        {
                            robot.robotBase.purePursuitDrive.start(
                                event, 0.0, false,
                                robot.robotInfo.baseParams.profiledMaxDriveVelocity,
                                robot.robotInfo.baseParams.profiledMaxDriveAcceleration,
                                robot.robotInfo.baseParams.profiledMaxDriveDeceleration,
                                robot.adjustPoseByAlliance(shootPose, autoChoices.alliance));
                        }
                        sm.waitForSingleEvent(event, State.SHOOT_ARTIFACTS);
                    }
                    break;

                case SHOOT_ARTIFACTS:
                    boolean pickUpLoading = autoChoices.loadingPickup == FtcAuto.LoadingPickup.YES && !(autoChoices.startPos == FtcAuto.StartPos.GOAL_ZONE) && !loadingPickedUp;
                    if (robot.autoShootTask != null)
                    {
                        robot.autoShootTask.autoShoot(
                            null, event, autoChoices.alliance, true, true, true,
                            currentSpikeMarkCount > 0 && autoChoices.classifierVision == FtcAuto.ClassifierVision.YES,
                            Dashboard.Subsystem_Shooter.autoShootParams.useRegression, true, false, 3, false);
                        sm.waitForSingleEvent(event, pickUpLoading ?
                                State.PICKUP_LOADING : State.PICKUP_SPIKEMARK);
                    }
                    else
                    {
                        sm.setState(pickUpLoading ?
                                State.PICKUP_LOADING : State.PICKUP_SPIKEMARK);
                    }
                    break;

                case PICKUP_LOADING:
                    if (robot.intakeSubsystem != null)
                    {
                        robot.intakeSubsystem.setBulldozeIntakeEnabled(true, 1.0, null);
                    }
                    robot.robotBase.driveBase.holonomicDrive(0.0, 0.45, 0.0, 3.0, event);
                    sm.waitForSingleEvent(event, State.MOVE_BACK);
                    break;

                case MOVE_BACK:
                    robot.robotBase.driveBase.holonomicDrive(0.0, -0.5, 0.0, 1.0, event);
                    sm.waitForSingleEvent(event, State.ATTEMPT_PICKUP);
                    break;

                case ATTEMPT_PICKUP:
                    robot.robotBase.driveBase.holonomicDrive(0.0, 0.35, 0.0, 3.0, event);
                    sm.waitForSingleEvent(event, State.FINISH_LOADING);
                    break;

                case FINISH_LOADING:
                    if (robot.intakeSubsystem != null)
                    {
                        robot.intakeSubsystem.setBulldozeIntakeEnabled(false, null, null);
                    }
                    loadingPickedUp = true;
                    sm.setState(State.GOTO_SHOOT_POS);
                    break;

                case PICKUP_SPIKEMARK:
                    if (currentSpikeMarkCount < targetSpikeMarkCount)
                    {
                        int spikeMarkIndex = spikeMarkOrder[currentSpikeMarkCount];

                        if (robot.intakeSubsystem != null)
                        {
                            spindexerFullEvent.clear();
                            sm.addEvent(spindexerFullEvent);
                            robot.intakeSubsystem.setBulldozeIntakeEnabled(true, 1.0, spindexerFullEvent);
                        }

                        TrcPose2D spikeMarkPose = robot.adjustPoseByAlliance(
                            RobotParams.Game.RED_SPIKEMARK_POSES[spikeMarkIndex], autoChoices.alliance);
                        TrcPose2D spikeMarkPoseAdj;

                        if (autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE)
                        {
                            spikeMarkPoseAdj = autoChoices.startPos == FtcAuto.StartPos.GOAL_ZONE?
                                RobotParams.Game.RED_SPIKEMARK_GOAL_ZONE_POSE_ADJS[spikeMarkIndex]:
                                RobotParams.Game.RED_SPIKEMARK_FAR_ZONE_POSE_ADJS[spikeMarkIndex];
                        }
                        else
                        {
                            spikeMarkPoseAdj = autoChoices.startPos == FtcAuto.StartPos.GOAL_ZONE?
                                RobotParams.Game.BLUE_SPIKEMARK_GOAL_ZONE_POSE_ADJS[spikeMarkIndex]:
                                RobotParams.Game.BLUE_SPIKEMARK_FAR_ZONE_POSE_ADJS[spikeMarkIndex];
                        }

                        spikeMarkPose.x += spikeMarkPoseAdj.x;
                        spikeMarkPose.y += spikeMarkPoseAdj.y;

                        TrcPose2D endPose = spikeMarkPose.clone();
                        endPose.y += autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE? 29.0: -29.0;
                        if (autoChoices.startPos == FtcAuto.StartPos.GOAL_ZONE)
                        {
                            if (currentSpikeMarkCount == 1 || currentSpikeMarkCount == 2)
                            {
                                endPose.y += autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE? 9.0: -9.0;
                            }
                        }
                        else
                        {
                            if (currentSpikeMarkCount == 0 || currentSpikeMarkCount == 1)
                            {
                                endPose.y += autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE? 9.0: -9.0;
                            }
                        }

                        robot.robotBase.purePursuitDrive.setWaypointEventHandler(
                            (i, wp) ->
                            {
                                robot.globalTracer.traceInfo(moduleName, "WaypointHandler: index=" + i);
                                if (i == 1)
                                {
                                    robot.robotBase.purePursuitDrive.setMoveOutputLimit(0.25);
                                }
                            });
                        robot.robotBase.purePursuitDrive.setMoveOutputLimit(1.0);
                        event.clear();
                        sm.addEvent(event);
                        // We will be moving slowly while picking up artifacts, disable stall detection so
                        // PurePursuit drive won't terminate prematurely near the end.
//                        robot.robotBase.purePursuitDrive.setStallDetectionEnabled(false);
                        robot.robotBase.purePursuitDrive.start(
                            // If there is no intake subsystem, there is no spindexerFullEvent. So, we must wait
                            // for PurePursuit drive event instead. This is for testing pathing only.
                            robot.intakeSubsystem == null? event: null, 0.0, false,
                            robot.robotInfo.baseParams.profiledMaxDriveVelocity,
                            robot.robotInfo.baseParams.profiledMaxDriveAcceleration,
                            robot.robotInfo.baseParams.profiledMaxDriveDeceleration,
                            spikeMarkPose, endPose);
                        sm.waitForEvents(State.FINISH_PICKUP, false, false, 5.5);
                    }
                    else
                    {
                        sm.setState(autoChoices.parkOption == FtcAuto.ParkOption.NO_PARK ? State.DONE : State.PARK);
                    }
                    break;

                case FINISH_PICKUP:
//                    robot.robotBase.purePursuitDrive.setStallDetectionEnabled(true);
                    robot.robotBase.purePursuitDrive.cancel();
                    robot.robotBase.purePursuitDrive.setWaypointEventHandler(null);
                    if (robot.intakeSubsystem != null)
                    {
                        robot.intakeSubsystem.setBulldozeIntakeEnabled(false, null, null);
                    }
                    currentSpikeMarkCount++;
                    sm.setState(
                        autoChoices.openGate == FtcAuto.OpenGate.YES && currentSpikeMarkCount == 1?
                            State.OPEN_GATE: State.GOTO_SHOOT_POS);
                    break;

                case OPEN_GATE:
                    robot.robotBase.purePursuitDrive.setMoveOutputLimit(0.85);
                    robot.robotBase.purePursuitDrive.start(
                        event, 0.0, false,
                        robot.robotInfo.baseParams.profiledMaxDriveVelocity,
                        robot.robotInfo.baseParams.profiledMaxDriveAcceleration,
                        robot.robotInfo.baseParams.profiledMaxDriveDeceleration,
                        robot.adjustPoseByAlliance(RobotParams.Game.RED_OPEN_GATE_POSE, autoChoices.alliance));
                    sm.waitForSingleEvent(event, State.GOTO_SHOOT_POS);
                    break;

                case PARK:
                    robot.robotBase.purePursuitDrive.setMoveOutputLimit(1.0);
                    robot.robotBase.purePursuitDrive.start(
                        event, 0.0, false,
                        robot.robotInfo.baseParams.profiledMaxDriveVelocity,
                        robot.robotInfo.baseParams.profiledMaxDriveAcceleration,
                        robot.robotInfo.baseParams.profiledMaxDriveDeceleration,
                        robot.adjustPoseByAlliance(
                            autoChoices.parkOption == FtcAuto.ParkOption.CLASSIFIER_PARK ?
                                RobotParams.Game.RED_ClASSIFIER_PARK_POSE:
                                RobotParams.Game.RED_SQUARE_PARK_POSE,
                            autoChoices.alliance));
                    sm.waitForSingleEvent(event, State.DONE);
                    break;

                case DONE:
                default:
                    // We are done.
                    cancel();
                    break;
            }
            robot.globalTracer.tracePostStateInfo(
                sm.toString(), state, robot.robotBase.driveBase, robot.robotBase.pidDrive,
                robot.robotBase.purePursuitDrive, null);
        }

        return !sm.isEnabled();
    }   //cmdPeriodic

}   //class CmdDecodeAuto
