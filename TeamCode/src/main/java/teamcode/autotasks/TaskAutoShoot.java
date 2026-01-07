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
import teamcode.FtcAuto;
import teamcode.Robot;
import teamcode.RobotParams;
import teamcode.indicators.LEDIndicator;
import teamcode.subsystems.Shooter;
import teamcode.vision.Vision;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcAutoTask;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcOwnershipMgr;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcTaskMgr;
import trclib.subsystem.TrcShootParams;
import trclib.timer.TrcTimer;
import trclib.vision.TrcVisionTargetInfo;

/**
 * This class implements auto-assist task.
 */
public class TaskAutoShoot extends TrcAutoTask<TaskAutoShoot.State>
{
    private static final String moduleName = TaskAutoShoot.class.getSimpleName();
    private static final boolean useGoalTracking = true;

    public enum State
    {
        START,
        DO_VISION,
        AIM,
        SHOOT,
        SHOOT_NEXT,
        NEXT_EXIT_SLOT,
        DONE
    }   //enum State

    public static class TaskParams
    {
        private FtcAuto.Alliance alliance = null;
        public boolean inAuto = false;
        public boolean doMotif = false;
        public boolean useClassifierVision = false;
        public boolean useRegression = false;
        public boolean flywheelTracking = true;
        public boolean relocalize = false;
        public int numArtifactsToShoot = 3;
        public boolean moveToNextExitSlot = true;

        public TaskParams setAlliance(FtcAuto.Alliance alliance)
        {
            this.alliance = alliance;
            return this;
        }   //setAlliance

        public TaskParams setInAuto(boolean inAuto)
        {
            this.inAuto = inAuto;
            return this;
        }   //setInAuto

        public TaskParams setVision(boolean doMotif, boolean useClassifierVision, boolean useRegression)
        {
            this.doMotif = doMotif;
            this.useClassifierVision = useClassifierVision;
            this.useRegression = useRegression;
            return this;
        }   //setVision

        public TaskParams setFlywheelTracking(boolean flywheelTracking)
        {
            this.flywheelTracking = flywheelTracking;
            return this;
        }   //setFlywheelTracking

        public TaskParams setRelocalizeEnabled(boolean enabled)
        {
            this.relocalize = enabled;
            return this;
        }   //setRelocalizeEnabled

        public TaskParams setNumArtifactsToShoot(int count, boolean moveToNextExitSlot)
        {
            this.numArtifactsToShoot = count;
            this.moveToNextExitSlot = moveToNextExitSlot;
            return this;
        }   //setNumArtifactsToShoot

        @NonNull
        public String toString()
        {
            return "(alliance=" + alliance +
                   ",inAuto=" + inAuto +
                   ",doMotif=" + doMotif +
                   ",useClassifierVision=" + useClassifierVision +
                   ",useRegression=" + useRegression +
                   ",flywheelTracking=" + flywheelTracking +
                   ",relocalize=" + relocalize +
                   ",numArtifactsToShoot=" + numArtifactsToShoot +
                   ",moveToNextExitSlot=" + moveToNextExitSlot + ")";
        }   //toString
    }   //class TaskParams

    private final TaskParams autoShootParams = new TaskParams();
    private final Robot robot;
    private final TrcEvent event;
    private final TrcEvent spindexerEvent;

    private int numArtifactsShot = 0;
    private int motifIndex = 0;
    private double[] aimInfo = null;
    private Vision.ArtifactType[] motifSequence = null;
    private Double visionExpiredTime = null;
    private boolean pausedPrevGoalTracking = false;

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
     * @param inAuto specifies true if running in Autonomous mode, false otherwise.
     * @param doMotif specifies true to shoot motif sequence.
     * @param useClassifierVision specifies true to use Classifier Vision, false otherwise, applicable only if doMotif
     *        is true.
     * @param useRegression specifies true to use polynomial regression determining shooter speed, false to use
     *        table lookup with linear interpolation/extrapolation.
     * @param flywheelTracking specifies true to spin the flywheel according to goal tracking.
     * @param relocalize specifies true to enable relocalization, false otherwise.
     * @param numArtifactsToShoot specifies the number of artifacts to shoot.
     * @param moveToNextExitSlot specifies true to move to next Exit slot after shooting is done.
     */
    public void autoShoot(
        String owner, TrcEvent completionEvent, FtcAuto.Alliance alliance, boolean inAuto, boolean doMotif,
        boolean useClassifierVision, boolean useRegression, boolean flywheelTracking, boolean relocalize,
        int numArtifactsToShoot, boolean moveToNextExitSlot)
    {
        autoShootParams
            .setAlliance(alliance)
            .setInAuto(inAuto)
            .setVision(doMotif, useClassifierVision, useRegression)
            .setFlywheelTracking(flywheelTracking)
            .setRelocalizeEnabled(relocalize)
            .setNumArtifactsToShoot(numArtifactsToShoot, moveToNextExitSlot);
        tracer.traceInfo(
            moduleName,
            "autoShoot(owner=" + owner + ", event=" + completionEvent + ", taskParams=" + autoShootParams + ")");
        if (robot.shooterSubsystem.isGoalTrackingEnabled())
        {
            robot.shooterSubsystem.pauseGoalTracking();
            pausedPrevGoalTracking = true;
        }
        startAutoTask(owner, State.START, autoShootParams, completionEvent);
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
               robot.spindexer.acquireExclusiveAccess(owner) &&
               robot.intake.acquireExclusiveAccess(owner);
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
                "\n\tspindexer=" + ownershipMgr.getOwner(robot.spindexer) +
                "\n\tintake=" + ownershipMgr.getOwner(robot.intake));
            robot.shooter.releaseExclusiveAccess(owner);
            robot.spindexer.releaseExclusiveAccess(owner);
            robot.intake.releaseExclusiveAccess(owner);
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
        if (robot.shooterSubsystem.isGoalTrackingEnabled())
        {
            // GoalTracking was enabled by this AutoTask, disable it.
            robot.shooterSubsystem.disableGoalTracking(owner);
        }

        if (pausedPrevGoalTracking)
        {
            // GoalTracking was enabled outside of this AutoTask, resume it.
            robot.shooterSubsystem.resumeGoalTracking();
            pausedPrevGoalTracking = false;
        }
        else
        {
            // GoalTracking was not ON at all, stop the shooter subsystem (stopping flywheel).
            robot.shooter.cancel(robot.shooter.getCurrentOwner());
        }

        robot.spindexer.cancel(owner);
        if (robot.ledIndicator != null)
        {
            robot.ledIndicator.setStatusPatternState(LEDIndicator.SEARCHING_RED_APRILTAG, false);
            robot.ledIndicator.setStatusPatternState(LEDIndicator.SEARCHING_BLUE_APRILTAG, false);
        }
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
        Vision.ArtifactType artifactType;

        robot.vision.detectClassifierArtifacts(taskParams.alliance);
        switch (state)
        {
            case START:
                if (robot.vision == null || !robot.vision.isLimelightVisionEnabled())
                {
                    tracer.traceWarn(moduleName, "***** Vision is not enabled, quit.");
                    sm.setState(State.DONE);
                }
                else
                {
                    TrcEvent turretCallbackEvent = null;
                    boolean classifierVisionEnabled = robot.vision.isClassifierVisionEnabled();

                    tracer.traceInfo(
                        moduleName,
                        "***** Setting up Vision (useClassifierVision=" + taskParams.useClassifierVision +
                        ", classifierVisionEnabled=" + classifierVisionEnabled + ")");
                    aimInfo = null;
                    numArtifactsShot = 0;
                    motifSequence = null;
                    visionExpiredTime = null;
                    if (useGoalTracking)
                    {
                        // Check if Limelight is facing the AprilTag.
                        synchronized (robot.trackingInfo)
                        {
                            if (robot.trackingInfo.robotLocalized)
                            {
                                double[] aimInfo = robot.vision.getAimInfoByOdometry(taskParams.alliance);
                                double turretAngle = robot.shooter.getPanAngle();
                                double turretTarget = aimInfo[1];

                                if (turretTarget < Shooter.Params.PAN_MIN_POS)
                                {
                                    turretTarget += 360.0;
                                }
                                else if (turretTarget > Shooter.Params.PAN_MAX_POS)
                                {
                                    turretTarget -= 360.0;
                                }
                                tracer.traceInfo(
                                    moduleName,
                                    "***** AimInfo=" + Arrays.toString(aimInfo) +
                                    ", turretAngle=" + turretAngle +
                                    ", turretTarget=" + turretTarget);
                                // Check turret target angle is greater than at least half of Limelight HFOV.
                                // If so, it means AprilTag is out-of-view and we need to turn the turret towards
                                // the AprilTag to bring it back in view.
                                if (Math.abs(turretTarget - turretAngle) > Vision.LIMELIGHT_HFOV_THRESHOLD)
                                {
                                    tracer.traceInfo(
                                        moduleName,
                                        "***** Camera is not pointing at AprilTag, turn to AprilTag.");
                                    turretCallbackEvent = new TrcEvent(moduleName + ".turretCallback");
                                    turretCallbackEvent.setCallback(
                                        (ctxt, canceled) ->
                                        {
                                            if (!canceled)
                                            {
                                                if (!robot.shooterSubsystem.isGoalTrackingEnabled())
                                                {
                                                    tracer.traceInfo(
                                                        moduleName,
                                                        "***** Camera is pointing at AprilTag, turn on Goal Tracking.");
                                                    robot.shooterSubsystem.enableGoalTracking(
                                                        owner, true, taskParams.alliance, true);
                                                }
                                            }
                                        }, null);
                                    robot.shooter.setPanAngle(owner, turretTarget, turretCallbackEvent, 0.0);
                                }
                            }
                            else
                            {
                                // Since robot is not localized, we can't determine if we are seeing AprilTag,
                                // don't turn on GoalTracking. This will force DO_VISION to use vision to find
                                // the AprilTag in case Limelight does see the AprilTag.
                                tracer.traceInfo(
                                    moduleName,
                                    "***** Robot is not localized, cannot determine if turret is seeing AprilTag.");
                            }
                        }
                    }

                    if (taskParams.useClassifierVision && !classifierVisionEnabled)
                    {
                        // Turn on ClassifierVision.
                        // Turn off ArtifactVision just in case it exists.
                        robot.vision.setArtifactVisionEnabled(Vision.ArtifactType.Any, false);
                        robot.vision.setClassifierVisionEnabled(true, true);
                    }

                    if (robot.ledIndicator != null)
                    {
                        // Clear other LED states so this will show.
                        robot.ledIndicator.setStatusPatternOn(
                            taskParams.alliance == FtcAuto.Alliance.BLUE_ALLIANCE?
                                LEDIndicator.SEARCHING_BLUE_APRILTAG: LEDIndicator.SEARCHING_RED_APRILTAG,
                            true);
                    }

                    if (turretCallbackEvent != null)
                    {
                        sm.waitForSingleEvent(turretCallbackEvent, State.DO_VISION);
                    }
                    else
                    {
                        sm.setState(State.DO_VISION);
                    }
                }
                break;

            case DO_VISION:
                if (!robot.shooterSubsystem.isGoalTrackingEnabled() && aimInfo == null)
                {
                    // Use vision to determine the appropriate AprilTag location.
                    int[] goalAprilTags =
                        taskParams.alliance == null? RobotParams.Game.anyGoalAprilTags:
                        taskParams.alliance == FtcAuto.Alliance.BLUE_ALLIANCE?
                            RobotParams.Game.blueGoalAprilTag: RobotParams.Game.redGoalAprilTag;
                    TrcVisionTargetInfo<FtcLimelightVision.DetectedObject> aprilTagInfo =
                        robot.vision.limelightVision.getBestDetectedTargetInfo(
                            FtcLimelightVision.ResultType.Fiducial, goalAprilTags, null);
                    if (aprilTagInfo != null)
                    {
                        int aprilTagId = (int) aprilTagInfo.detectedObj.objId;
                        // getObjectPose clones the pose, so we are safe to modify it.
                        TrcPose2D aprilTagPose = aprilTagInfo.detectedObj.getObjectPose();
                        tracer.traceInfo(
                            moduleName, "***** Vision found AprilTag " + aprilTagId + ": aprilTagPose=" + aprilTagPose);

                        if (robot.ledIndicator != null)
                        {
                            // This is assuming vision is set to look for 20 or 24 only.
                            robot.ledIndicator.setStatusPatternOn(
                                aprilTagId == RobotParams.Game.blueGoalAprilTag[0]?
                                    LEDIndicator.BLUE_APRILTAG: LEDIndicator.RED_APRILTAG, true);
                        }

                        if (useGoalTracking && !robot.shooterSubsystem.isGoalTrackingEnabled())
                        {
                            tracer.traceInfo(moduleName, "***** Camera found AprilTag, turn on Goal Tracking.");
                            robot.shooterSubsystem.enableGoalTracking(owner, true, taskParams.alliance, true);
                        }
                        aimInfo = robot.vision.getAimInfoByVision(aprilTagInfo);
                    }
                }

                // If we are doing motif, determine the shooting sequence.
                if (taskParams.doMotif && robot.obeliskMotif != null && motifSequence == null)
                {
                    // If we saw the obelisk and haven't determined the motif sequence, determine it now.
                    // Do this only once since DO_VISION gets called repeatedly.
                    robot.vision.resetClassifierArtifactsFilter();
                    motifSequence = robot.vision.getMotifSequence(
                        taskParams.alliance, robot.obeliskMotif, taskParams.useClassifierVision);
                    motifIndex = 0;
                    tracer.traceInfo(
                        moduleName, "***** MotifSequence=%s", Arrays.toString(motifSequence));
                }

                if ((aimInfo != null || robot.shooterSubsystem.isGoalTrackingEnabled()) &&
                    (!taskParams.doMotif || robot.obeliskMotif == null || motifSequence != null))
                {
                    // Vision found the target AprilTag and either we don't see obelisk or we have determined
                    // the motif sequence, go ahead and shoot. If vision saw the obelisk but it may take time
                    // to determine the motif sequence if we are using Classifier Vision, then we wait for it.
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
                        robot.ledIndicator.setStatusPatternOn(LEDIndicator.NOT_FOUND, true);
                    }
                    sm.setState(State.DONE);
                }
                break;

            case AIM:
                // Determine what artifact type to shoot and check if Spindexer has it.
                sm.clearEventList();
                Vision.ArtifactType motifArtifactType = motifSequence != null? motifSequence[motifIndex++]: null;
                if (motifArtifactType != null)
                {
                    artifactType = motifArtifactType;
                    if (robot.spindexerSubsystem.getNumArtifacts(artifactType) == 0)
                    {
                        tracer.traceInfo(moduleName, "***** %s motif artifact not in Spindexer.", artifactType);
                        // Pick another artifact type to shoot. Ideally not the same type as the next motif sequence.
                        artifactType =
                            motifIndex == motifSequence.length ? Vision.ArtifactType.Any :
                            motifSequence[motifIndex] == Vision.ArtifactType.Green ?
                                Vision.ArtifactType.Purple : Vision.ArtifactType.Green;
                        if (artifactType == motifArtifactType)
                        {
                            // Don't pick the same color that we already know we don't have. In that case, just
                            // shoot any color.
                            artifactType = Vision.ArtifactType.Any;
                        }
                        if (robot.spindexerSubsystem.getNumArtifacts(artifactType) == 0 &&
                            robot.spindexerSubsystem.getNumArtifacts(Vision.ArtifactType.Any) == 0)
                        {
                            tracer.traceInfo(
                                moduleName, "***** Spindexer is empty, shoot any just in case sensor is wrong.");
                            artifactType = Vision.ArtifactType.Any;
                        }
                    }
                }
                else
                {
                    artifactType = Vision.ArtifactType.Any;
                }
                // Spin the Spindexer to the exit slot that contains the artifact we want.
                Vision.ArtifactType exitArtifactType = robot.spindexerSubsystem.getExitSlotArtifactType();
                tracer.traceInfo(
                    moduleName, "***** Shooting %s artifact (MotifArtifact=%s, exitArtifact=%s).",
                    artifactType, motifArtifactType, exitArtifactType);
                // Move Spindexer to the slot that has the correct artifact type.
                spindexerEvent.clear();
                sm.addEvent(spindexerEvent);
                if (!robot.spindexerSubsystem.moveToExitSlotWithArtifact(owner, artifactType, spindexerEvent) &&
                    exitArtifactType == null)
                {
                    // Spindexer did not move, probably because it is empty or it doesn't know what artifacts it has.
                    // Exit slot is not aligned, move to the next exit slot up so the launcher won't get stuck.
                    // Clear spindexer event again since moveToExitSlotWithArtifact would have signaled it.
                    spindexerEvent.clear();
                    robot.spindexerSubsystem.exitSlotUp(owner, spindexerEvent);
                }

                if (robot.shooterSubsystem.isGoalTrackingEnabled())
                {
                    event.clear();
                    sm.addEvent(event);
                    tracer.traceInfo(moduleName, "***** Waiting for shooter ready (event=" + event + ")");
                    robot.shooterSubsystem.waitForShooterReady(event);
                }
                else if (aimInfo != null)
                {
                    // GoalTracking is not ON but vision detected AprilTag, we can aim according to vision info.
                    // Spin the shooter flywheel up to speed and the turret pointing to the target.
                    event.clear();
                    sm.addEvent(event);
                    TrcShootParams.Entry shootParams = Shooter.shootParamsTable.get(
                        aimInfo[0], taskParams.useRegression);
                    tracer.traceInfo(
                        moduleName, "***** ShootParams: dist=%f, bearing=%f, shootParams=%s, event=%s",
                        aimInfo[0], aimInfo[1], shootParams, event);
                    robot.shooter.setTiltAngle(shootParams.region.tiltAngle);
                    robot.shooter.aimShooter(
                        owner, shootParams.outputs[0]/60.0, 0.0, null, aimInfo[1], event, 0.0, null, 0.0);
                }
                // GoalTracking is not ON and vision has detected AprilTag but aimInfo was cleared for faster
                // shooting. It doesn't need to wait for flywheel speed recovery.
                sm.waitForEvents(State.SHOOT, false, true);
                break;

            case SHOOT:
                tracer.traceInfo(
                    moduleName,
                    "***** Shooting artifact " + (numArtifactsShot + 1) +
                    " (numArtifactsLeft=" + taskParams.numArtifactsToShoot + ")");
                robot.shooterSubsystem.shoot(owner, event);
                sm.waitForSingleEvent(event, State.SHOOT_NEXT);
                break;

            case SHOOT_NEXT:
                numArtifactsShot++;
                taskParams.numArtifactsToShoot--;
                // Clearing aimInfo to null forcing the AIM code to not wait for flywheel speed recovery.
                // This will shoot faster assuming flywheel speed has recovered enough.
                // This is only relevant if GoalTracking is not ON.
//                aimInfo = null;
                sm.setState(
                    taskParams.numArtifactsToShoot > 0 &&
                    robot.spindexerSubsystem.getNumArtifacts(Vision.ArtifactType.Any) > 0?
                        State.AIM: State.NEXT_EXIT_SLOT);
                break;

            case NEXT_EXIT_SLOT:
                if (taskParams.moveToNextExitSlot)
                {
                    // If spindexer is empty, moveToExitSlotWithArtifact will fail and will immediately signal the
                    // event before we call waitForSingleEvent. Therefore, we need to tell waitForSingleEvent to not
                    // clear the event and we need to manually clear the event beforehand.
                    event.clear();
                    tracer.traceInfo(moduleName, "***** Move to the next exit slot.");
                    robot.spindexerSubsystem.moveToExitSlotWithArtifact(owner, Vision.ArtifactType.Any, event);
                    sm.waitForSingleEvent(event, State.DONE, false, 0.0);
                }
                else
                {
                    sm.setState(State.DONE);
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
