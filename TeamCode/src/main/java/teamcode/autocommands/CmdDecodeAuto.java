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
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcStateMachine;
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
        GOTO_PRELOAD_SHOOT_POS,
        SHOOT_PRELOAD,
        PICKUP_SPIKEMARK,
        FIND_MOTIF,
        SHOOT_SPIKEMARK,
        PARK,
        DONE
    }   //enum State

    private final Robot robot;
    private final FtcAuto.AutoChoices autoChoices;
    private final TrcTimer timer;
    private final TrcEvent event;
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
                    targetSpikeMarkCount = (int) autoChoices.spikeMarkCount;
                    // Do delay if necessary.
                    if (autoChoices.startDelay > 0.0)
                    {
                        robot.globalTracer.traceInfo(
                            moduleName, "***** Do Start Delay " + autoChoices.startDelay + "s.");
                        timer.set(autoChoices.startDelay, event);
                        sm.waitForSingleEvent(event, State.GOTO_PRELOAD_SHOOT_POS);
                    }
                    else
                    {
                        sm.setState(State.GOTO_PRELOAD_SHOOT_POS);
                    }
                    break;

                case GOTO_PRELOAD_SHOOT_POS:
                    robot.shooter.panMotor.setPosition(
                            null, 0.0, -180.0, true, Shooter.Params.PAN_POWER_LIMIT, null, 0.0);
                    robot.robotDrive.purePursuitDrive.start(
                        event, 0.0, false,
                        robot.robotInfo.baseParams.profiledMaxDriveVelocity,
                        robot.robotInfo.baseParams.profiledMaxDriveAcceleration,
                        robot.robotInfo.baseParams.profiledMaxDriveDeceleration,
                        robot.adjustPoseByAlliance(
                            autoChoices.startPos == FtcAuto.StartPos.GOAL_ZONE ?
                                RobotParams.Game.RED_PRELOAD_GOAL_SHOOT_POSE :
                                RobotParams.Game.RED_PRELOAD_LAUNCH_SHOOT_POSE,
                            autoChoices.alliance));
                    sm.waitForSingleEvent(event, State.SHOOT_PRELOAD);
                    break;

                case SHOOT_PRELOAD:
                    robot.autoShootTask.autoShoot(null, event, autoChoices.alliance, true, false, 3, false);
                    sm.waitForSingleEvent(event, State.PICKUP_SPIKEMARK);
                    break;

                case PICKUP_SPIKEMARK:
                    int[] order = (autoChoices.startPos == FtcAuto.StartPos.GOAL_ZONE) ?
                            new int[]{0, 1, 2} : new int[]{2, 1, 0};
                    if (currentSpikeMarkCount < targetSpikeMarkCount)
                    {
                        int index = order[currentSpikeMarkCount];
                        TrcPose2D targetPose = robot.adjustPoseByAlliance(
                            RobotParams.Game.RED_SPIKEMARK_POS[index], autoChoices.alliance);
                        robot.robotDrive.purePursuitDrive.start(event, 0.0, false,
                                robot.robotInfo.baseParams.profiledMaxDriveVelocity,
                                robot.robotInfo.baseParams.profiledMaxDriveAcceleration,
                                robot.robotInfo.baseParams.profiledMaxDriveDeceleration,
                                targetPose);
                        robot.intakeSubsystem.setBulldozeIntakeEnabled(true);
                        robot.robotDrive.purePursuitDrive.start(event, 0.0, true,
                                robot.robotInfo.baseParams.profiledMaxDriveVelocity,
                                robot.robotInfo.baseParams.profiledMaxDriveAcceleration,
                                robot.robotInfo.baseParams.profiledMaxDriveDeceleration,
                                new TrcPose2D(0.0, 20.0, 0.0)); // TODO: tune
                        robot.intakeSubsystem.setBulldozeIntakeEnabled(false);
                        currentSpikeMarkCount++;
                        sm.waitForSingleEvent(event, State.SHOOT_SPIKEMARK);
                    }
                    else
                    {
                        sm.setState(autoChoices.parkOption == FtcAuto.ParkOption.PARK ? State.PARK : State.DONE);
                    }
                    break;

                case FIND_MOTIF:
                    // TODO: Add code to check for motif
                    break;

                case SHOOT_SPIKEMARK:
                    robot.robotDrive.purePursuitDrive.start(event, 0.0, false,
                            robot.robotInfo.baseParams.profiledMaxDriveVelocity,
                            robot.robotInfo.baseParams.profiledMaxDriveAcceleration,
                            robot.robotInfo.baseParams.profiledMaxDriveDeceleration,
                            robot.adjustPoseByAlliance(
                                RobotParams.Game.RED_SPIKEMARK_SHOOT_POSE, autoChoices.alliance));
                    robot.autoShootTask.autoShoot(null, event, autoChoices.alliance, true, true, 3, false);
                    sm.waitForSingleEvent(event, State.PICKUP_SPIKEMARK);
                    break;

                case PARK:
                    robot.robotDrive.purePursuitDrive.start(event, 0.0, false,
                            robot.robotInfo.baseParams.profiledMaxDriveVelocity,
                            robot.robotInfo.baseParams.profiledMaxDriveAcceleration,
                            robot.robotInfo.baseParams.profiledMaxDriveDeceleration,
                            robot.adjustPoseByAlliance(RobotParams.Game.RED_PARK_POSE, autoChoices.alliance));
                    sm.waitForSingleEvent(event, State.DONE);
                    break;

                case DONE:
                default:
                    // We are done.
                    cancel();
                    break;
            }
            robot.globalTracer.tracePostStateInfo(
                sm.toString(), state, robot.robotDrive.driveBase, robot.robotDrive.pidDrive,
                robot.robotDrive.purePursuitDrive, null);
        }

        return !sm.isEnabled();
    }   //cmdPeriodic

}   //class CmdDecodeAuto
