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

package teamcode.autotasks;

import androidx.annotation.NonNull;

import java.util.Arrays;

import ftclib.vision.FtcLimelightVision;
import teamcode.Dashboard;
import teamcode.FtcAuto;
import teamcode.Robot;
import teamcode.subsystems.LEDIndicator;
import teamcode.subsystems.Shooter;
import teamcode.vision.Vision;
import trclib.dataprocessor.TrcUtil;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcAutoTask;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcOwnershipMgr;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcTaskMgr;
import trclib.subsystem.TrcShootParamTable;
import trclib.timer.TrcTimer;
import trclib.vision.TrcVisionTargetInfo;

/**
 * This class implements auto-assist task.
 */
public class TaskAutoShoot extends TrcAutoTask<TaskAutoShoot.State>
{
    private static final String moduleName = TaskAutoShoot.class.getSimpleName();

    public enum State
    {
        START,
        DO_VISION,
        AIM,
        SHOOT,
        SHOOT_NEXT,
        DONE
    }   //enum State

    private static class TaskParams
    {
        FtcAuto.Alliance alliance;
        boolean useAprilTagVision;
        boolean useClassifierVision;
        int numArtifactsToShoot;
        int[] aprilTagIds;

        TaskParams(
            FtcAuto.Alliance alliance, boolean useAprilTagVision, boolean useClassifierVision, int numArtifactsToShoot,
            int... aprilTagIds)
        {
            this.alliance = alliance;
            this.useAprilTagVision = useAprilTagVision;
            this.useClassifierVision = useClassifierVision;
            this.numArtifactsToShoot = numArtifactsToShoot;
            this.aprilTagIds = aprilTagIds;
        }   //TaskParams

        @NonNull
        public String toString()
        {
            return "(alliance=" + alliance +
                   ",useAprilTagVision=" + useAprilTagVision +
                   ",useClassifierVision=" + useClassifierVision +
                   ",numArtifactsToShoot=" + numArtifactsToShoot +
                   ",aprilTagIds=" + Arrays.toString(aprilTagIds) + ")";
        }   //toString
    }   //class TaskParams

    private final Robot robot;
    private final TrcEvent event;
    private final TrcEvent spindexerEvent;

    private Double visionExpiredTime = null;
    TrcPose2D aprilTagPose = null;
    TrcShootParamTable.Params shootParams = null;
    Vision.ArtifactType[] motifSequence = null;
    int motifIndex = 0;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object that contains all the necessary subsystems.
     */
    public TaskAutoShoot(Robot robot)
    {
        super(moduleName, TrcTaskMgr.TaskType.POST_PERIODIC_TASK);
        this.robot = robot;
        this.event = new TrcEvent(moduleName + ".event");
        this.spindexerEvent = new TrcEvent(moduleName + ".spindexerEvent");
    }   //TaskAutoShoot

    /**
     * This method starts the auto-assist operation.
     *
     * @param owner specifies the owner to acquire subsystem ownerships, can be null if not requiring ownership.
     * @param completionEvent specifies the event to signal when done, can be null if none provided.
     * @param alliance specifies the alliance color, can be null if useClassifierVision is false.
     * @param useAprilTagVision specifies true to use AprilTag Vision, false otherwise.
     * @param useClassifierVision specifies true to use Classifier Vision, false otherwise.
     * @param numArtifactsToShoot specifies the number of artifacts to shoot.
     * @param aprilTagIds specifies multiple AprilTag IDs for vision to aim for, can be null to look for any AprilTag.
     */
    public void autoShoot(
        String owner, TrcEvent completionEvent, FtcAuto.Alliance alliance, boolean useAprilTagVision,
        boolean useClassifierVision, int numArtifactsToShoot, int... aprilTagIds)
    {
        TaskParams taskParams =
            new TaskParams(alliance, useAprilTagVision, useClassifierVision, numArtifactsToShoot, aprilTagIds);
        tracer.traceInfo(
            moduleName,
            "autoShoot(owner=" + owner + ", event=" + completionEvent + ", taskParams=" + taskParams + ")");
        startAutoTask(owner, State.START, taskParams, completionEvent);
    }   //autoShoot

    //
    // Implement TrcAutoTask abstract methods.
    //

    /**
     * This method is called to acquire ownership of all subsystems involved in the auto task operation. This is
     * typically called before starting an auto task operation.
     *
     * @param owner specifies the owner to acquire the subsystem ownerships.
     * @return true if acquired all subsystems ownership, false otherwise. It releases all ownership if any acquire
     *         failed.
     */
    @Override
    protected boolean acquireSubsystemsOwnership(String owner)
    {
        return owner == null ||
               robot.shooter.acquireExclusiveAccess(owner) &&
               robot.spindexer.acquireExclusiveAccess(owner);
    }   //acquireSubsystemsOwnership

    /**
     * This method is called to release ownership of all subsystems involved in the auto task operation. This is
     * typically called if the auto task operation is completed or canceled.
     *
     * @param owner specifies the owner that acquired the subsystem ownerships.
     */
    @Override
    protected void releaseSubsystemsOwnership(String owner)
    {
        if (owner != null)
        {
            TrcOwnershipMgr ownershipMgr = TrcOwnershipMgr.getInstance();
            tracer.traceInfo(
                moduleName,
                "Releasing subsystem ownership on behalf of " + owner +
                "\n\tshooter=" + ownershipMgr.getOwner(robot.shooter) +
                "\n\tspindexer=" + ownershipMgr.getOwner(robot.spindexer));
            robot.shooter.releaseExclusiveAccess(owner);
            robot.spindexer.releaseExclusiveAccess(owner);
        }
    }   //releaseSubsystemsOwnership

    /**
     * This method is called to stop all the subsystems. This is typically called if the auto task operation is
     * completed or canceled.
     *
     * @param owner specifies the owner that acquired the subsystem ownerships.
     */
    @Override
    protected void stopSubsystems(String owner)
    {
        tracer.traceInfo(moduleName, "Stopping subsystems.");
        robot.shooter.cancel(owner);
        robot.spindexer.cancel(owner);
    }   //stopSubsystems

    /**
     * This methods is called periodically to run the auto-assist task.
     *
     * @param owner specifies the owner that acquired the subsystem ownerships.
     * @param params specifies the task parameters.
     * @param state specifies the current state of the task.
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the competition mode (e.g. Autonomous, TeleOp, Test).
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false if running the fast loop on the main robot thread.
     */
    @Override
    protected void runTaskState(
        String owner, Object params, State state, TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode,
        boolean slowPeriodicLoop)
    {
        TaskParams taskParams = (TaskParams) params;

        switch (state)
        {
            case START:
                aprilTagPose = null;
                if (!taskParams.useAprilTagVision)
                {
                    // Not using AprilTag vision, skip vision and just score at current positon assuming operator
                    // has manually aimed at target.
                    tracer.traceInfo(moduleName, "***** Not using AprilTag Vision.");
                    sm.setState(State.AIM);
                }
                else if (robot.vision != null && robot.vision.isLimelightVisionEnabled() &&
                         (!taskParams.useClassifierVision || robot.vision.isClassifierVisionEnabled()))
                {
                    tracer.traceInfo(moduleName, "***** Using Vision.");
                    visionExpiredTime = null;
                    sm.setState(State.DO_VISION);
                }
                else
                {
                    tracer.traceInfo(moduleName, "***** Using AprilTag Vision but Vision is not enabled.");
                    sm.setState(State.DONE);
                }
                break;

            case DO_VISION:
                // Use vision to determine the appropriate AprilTag location.
                if (aprilTagPose == null)
                {
                    TrcVisionTargetInfo<FtcLimelightVision.DetectedObject> aprilTagInfo =
                        robot.vision.limelightVision.getBestDetectedTargetInfo(
                            FtcLimelightVision.ResultType.Fiducial, taskParams.aprilTagIds, null, null);
                    if (aprilTagInfo != null)
                    {
                        int aprilTagId = (int) aprilTagInfo.detectedObj.objId;
                        aprilTagPose = aprilTagInfo.detectedObj.getObjectPose();
                        tracer.traceInfo(
                            moduleName,
                            "***** Vision found AprilTag " + aprilTagId + ": aprilTagPose=" + aprilTagPose);
                        if (robot.ledIndicator != null)
                        {
                            // Indicate we timed out and found nothing.
                            robot.ledIndicator.setStatusPattern(LEDIndicator.APRIL_TAG);
                        }
                        // Determine shooter speed, pan and tilt angle according to detected AprilTag pose.
                        // Use vision distance to look up shooter parameters.
                        double aprilTagDistance = TrcUtil.magnitude(aprilTagPose.x, aprilTagPose.y);
                        shootParams = Shooter.Params.shootParamTable.get(aprilTagDistance, false);
                        tracer.traceInfo(
                            moduleName, "***** ShootParams: distance=" + aprilTagDistance + ", params=" + shootParams);
                    }
                }

                if (taskParams.useClassifierVision && robot.motif != null && motifSequence == null)
                {
                    Vision.ArtifactType[] classifierArtifacts =
                        robot.vision.getClassifierArtifacts(taskParams.alliance);
                    if (classifierArtifacts != null)
                    {
                        int noneIndex = -1;
                        for (int i = 0; i < classifierArtifacts.length; i++)
                        {
                            if (classifierArtifacts[i] == Vision.ArtifactType.None)
                            {
                                tracer.traceInfo(moduleName, "***** First classifier empty slot=" + noneIndex);
                                noneIndex = i % robot.motif.length;
                                break;
                            }
                        }

                        if (noneIndex != -1)
                        {
                            motifSequence = new Vision.ArtifactType[3];
                            for (int i = 0; i < motifSequence.length; i++)
                            {
                                motifSequence[i] = robot.motif[noneIndex];
                                noneIndex = (noneIndex + 1) % robot.motif.length;
                            }
                            motifIndex = 0;
                            tracer.traceInfo(
                                moduleName, "***** MotifSequence=" + Arrays.toString(motifSequence));
                        }
                    }
                }

                if (aprilTagPose != null &&
                    (!taskParams.useClassifierVision || robot.motif == null || motifSequence != null))
                {
                    sm.setState(State.AIM);
                }
                else if (visionExpiredTime == null)
                {
                    // Can't find AprilTag, set a timeout and try again.
                    visionExpiredTime = TrcTimer.getCurrentTime() + 1.0;
                }
                else if (TrcTimer.getCurrentTime() >= visionExpiredTime)
                {
                    // Timed out, moving on.
                    tracer.traceInfo(moduleName, "***** Vision timed out.");
                    if (robot.ledIndicator != null)
                    {
                        // Indicate we timed out and found nothing.
                        robot.ledIndicator.setStatusPattern(LEDIndicator.NOT_FOUND);
                    }
                    sm.setState(State.DONE);
                }
                break;

            case AIM:
                event.clear();
                sm.addEvent(event);
                if (shootParams != null)
                {
                    robot.shooter.aimShooter(
                        owner, shootParams.shooter1Velocity, shootParams.shooter2Velocity, shootParams.tiltAngle,
                        aprilTagPose.angle, event, 0.0, null, 0.0);
                }
                else
                {
                    // We did not use vision, just shoot assuming operator manually aimed.
                    double shooterVel = Dashboard.SubsystemShooter.shootMotor1Velocity;
                    tracer.traceInfo(moduleName, "***** ManualShoot: vel=" + shooterVel + " RPM");
                    // ShooterVel is in RPM, aimShooter wants RPS.
                    robot.shooter.aimShooter(
                        owner, shooterVel / 60.0, 0.0, null, null, event, 0.0, null, 0.0);
                }

                spindexerEvent.clear();
                sm.addEvent(spindexerEvent);
                if (motifSequence != null)
                {
                    robot.spindexerSubsystem.moveToExitSlotWithArtifact(
                        owner, motifSequence[motifIndex++], spindexerEvent);
                }
                else
                {
                    robot.spindexerSubsystem.moveToExitSlotWithArtifact(owner, Vision.ArtifactType.Any, spindexerEvent);
                }
                sm.waitForEvents(State.SHOOT, false, true);
                break;

            case SHOOT:
                robot.shooterSubsystem.shoot(owner, event);
                sm.waitForSingleEvent(event, State.SHOOT_NEXT);
                break;

            case SHOOT_NEXT:
                taskParams.numArtifactsToShoot--;
                if (taskParams.numArtifactsToShoot == 0)
                {
                    sm.setState(State.DONE);
                }
                else
                {
                    sm.setState(State.AIM);
                }
                break;

            case DONE:
            default:
                // Stop task.
                stopAutoTask(true);
                break;
        }
    }   //runTaskState
 
}   //class TaskAutoShoot
