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

import teamcode.FtcAuto;
import teamcode.Robot;
import teamcode.RobotParams;
import teamcode.subsystems.Shooter;
import teamcode.vision.Vision;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcStateMachine;
import trclib.subsystem.TrcShootParamTable;
import trclib.timer.TrcTimer;

/**
 * This class implements an autonomous strategy.
 */
public class CmdDecodeAuto implements TrcRobot.RobotCommand
{
    private static final String moduleName = CmdDecodeAuto.class.getSimpleName();

    private enum State
    {
        START,
        GOTO_SHOOT_POS,
        SHOOT_ARTIFACTS,
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

    private int currentSpikeMarkCount = 0;
    private int targetSpikeMarkCount = 0;

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
            robot.intakeSubsystem.setBulldozeIntakeEnabled(false, null);
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
                    targetSpikeMarkCount = (int) autoChoices.spikeMarkCount;
                    currentSpikeMarkCount = 0;
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
                    if (robot.shooterSubsystem != null)
                    {
                        // Pre-spin flywheel and set up pan/tilt angles for scoring artifacts (fire and forget).
                        TrcShootParamTable.Params shootParams;
                        double panAngle;
                        if (autoChoices.startPos == FtcAuto.StartPos.GOAL_ZONE)
                        {
                            shootParams = Shooter.Params.shootParamTable.get(Shooter.GOAL_ZONE_SHOOT_POINT);
                            panAngle = autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE ? -45.0 : 45.0;
                        }
                        else
                        {
                            shootParams = Shooter.Params.shootParamTable.get(Shooter.FAR_ZONE_SHOOT_POINT);
                            panAngle = autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE ? -20.0 : 20.0;
                        }
                        robot.shooter.aimShooter(
                            shootParams.shooter1Velocity/60.0, 0.0, shootParams.tiltAngle, panAngle);
                    }

                    if (autoChoices.startPos != FtcAuto.StartPos.GOAL_ZONE && currentSpikeMarkCount == 0)
                    {
                        sm.setState(State.SHOOT_ARTIFACTS);
                    }
                    else
                    {
                        TrcPose2D shootPose = autoChoices.startPos == FtcAuto.StartPos.GOAL_ZONE?
                            RobotParams.Game.RED_GOAL_ZONE_SHOOT_POSE: RobotParams.Game.RED_FAR_ZONE_SHOOT_POSE;
                        robot.robotBase.purePursuitDrive.setMoveOutputLimit(1.0);
                        robot.robotBase.purePursuitDrive.start(
                            event, 0.0, false,
                            robot.robotInfo.baseParams.profiledMaxDriveVelocity,
                            robot.robotInfo.baseParams.profiledMaxDriveAcceleration,
                            robot.robotInfo.baseParams.profiledMaxDriveDeceleration,
                            robot.adjustPoseByAlliance(shootPose, autoChoices.alliance));
                        sm.waitForSingleEvent(event, State.SHOOT_ARTIFACTS);
                    }
                    break;

                case SHOOT_ARTIFACTS:
                    if (robot.autoShootTask != null)
                    {
                        robot.autoShootTask.autoShoot(
                            null, event, autoChoices.alliance, true, true, true, false, false, 3, false);
                        sm.waitForSingleEvent(event, State.PICKUP_SPIKEMARK);
                    }
                    else
                    {
                        sm.setState(State.PICKUP_SPIKEMARK);
                    }
                    break;

                case PICKUP_SPIKEMARK:
                    if (currentSpikeMarkCount < targetSpikeMarkCount)
                    {
                        int[] order =
                            autoChoices.startPos == FtcAuto.StartPos.GOAL_ZONE? new int[] {0, 1, 2}:
                            autoChoices.openGate == FtcAuto.OpenGate.YES? new int[] {1, 0, 2}: new int[] {2, 1, 0};
                        int spikeMarkIndex = order[currentSpikeMarkCount];

                        if (robot.intakeSubsystem != null)
                        {
                            spindexerFullEvent.clear();
                            sm.addEvent(spindexerFullEvent);
                            robot.intakeSubsystem.setBulldozeIntakeEnabled(true, spindexerFullEvent);
                        }

                        TrcPose2D spikeMarkPose = robot.adjustPoseByAlliance(
                            RobotParams.Game.RED_SPIKEMARK_POSES[spikeMarkIndex], autoChoices.alliance);
                        TrcPose2D endPose = spikeMarkPose.clone();
                        endPose.y += autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE? 26.0: -26.0;
                        robot.robotBase.purePursuitDrive.setWaypointEventHandler(
                            (i, wp) ->
                            {
                                robot.globalTracer.traceInfo(moduleName, "WaypointHandler: index=" + i);
                                if (i == 1)
                                {
                                    robot.robotBase.purePursuitDrive.setMoveOutputLimit(0.2);
                                }
                            });
                        robot.robotBase.purePursuitDrive.setMoveOutputLimit(1.0);
                        event.clear();
                        sm.addEvent(event);
                        robot.robotBase.purePursuitDrive.start(
                            event, 0.0, false,
                            robot.robotInfo.baseParams.profiledMaxDriveVelocity,
                            robot.robotInfo.baseParams.profiledMaxDriveAcceleration,
                            robot.robotInfo.baseParams.profiledMaxDriveDeceleration,
                            spikeMarkPose, endPose);
                        sm.waitForEvents(State.FINISH_PICKUP, false, false);
                    }
                    else
                    {
                        sm.setState(autoChoices.parkOption == FtcAuto.ParkOption.NO_PARK ? State.DONE : State.PARK);
                    }
                    break;

                case FINISH_PICKUP:
                    robot.robotBase.purePursuitDrive.cancel();
                    robot.robotBase.purePursuitDrive.setWaypointEventHandler(null);
                    if (robot.intakeSubsystem != null)
                    {
                        robot.intakeSubsystem.setBulldozeIntakeEnabled(false, null);
                    }
                    currentSpikeMarkCount++;
                    sm.setState(autoChoices.openGate == FtcAuto.OpenGate.YES && currentSpikeMarkCount == 1?
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
