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

package teamcode.vision;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.Locale;

import ftclib.driverio.FtcDashboard;
import ftclib.robotcore.FtcOpMode;
import ftclib.vision.FtcEocvColorBlobProcessor;
import ftclib.vision.FtcLimelightVision;
import ftclib.vision.FtcVision;
import ftclib.vision.FtcVisionAprilTag;
import ftclib.vision.FtcVisionEocvColorBlob;
import teamcode.FtcAuto;
import teamcode.Robot;
import teamcode.RobotParams;
import teamcode.indicators.LEDIndicator;
import trclib.dataprocessor.TrcUtil;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcDbgTrace;
import trclib.vision.TrcHomographyMapper;
import trclib.vision.TrcOpenCvColorBlobPipeline;
import trclib.vision.TrcOpenCvDetector;
import trclib.vision.TrcVision;
import trclib.vision.TrcVisionTargetInfo;

/**
 * This class implements AprilTag/Eocv/Limelight Vision for the game season. It creates and initializes all the vision
 * target info as well as providing info for the robot, camera and the field. It also provides methods to get the
 * location of the robot and detected targets.
 */
public class Vision
{
    private final String moduleName = getClass().getSimpleName();

    // Lens properties for various cameras.
    private static final TrcOpenCvDetector.LensInfo logitechC920At640x480 =
        new TrcOpenCvDetector.LensInfo()
            .setLensProperties(622.001, 622.001, 319.803, 241.251)
            .setDistortionCoefficents(0.1208, -0.261599, 0, 0, 0.10308, 0, 0, 0);
    private static final TrcOpenCvDetector.LensInfo logitechC270At640x480 =
        new TrcOpenCvDetector.LensInfo()
            .setLensProperties(822.317, 822.317, 319.495, 242.502)
            .setDistortionCoefficents(-0.0449369, 1.17277, 0, 0, -3.63244, 0, 0, 0);
    private static final TrcOpenCvDetector.LensInfo lifeCamHD3000At640x480 =
        new TrcOpenCvDetector.LensInfo()
            .setLensProperties(678.154, 678.170, 318.135, 228.374)
            .setDistortionCoefficents(0.154576, -1.19143, 0, 0, 2.06105, 0, 0, 0);

    // Front camera properties
    public static final TrcVision.CameraInfo frontCamParams = new TrcVision.CameraInfo()
        .setCameraInfo("Webcam 1", 320, 240)
        .setCameraPose(0.0, 8.75, 11.0, 0.0, 0.0, 0.0)
        .setLensProperties(logitechC920At640x480)   // TODO: Need to calibrate camera for 320x480 for SolvePnp
        .setHomographyParams(
            new TrcHomographyMapper.Rectangle(
                14.0, 28.0,                     // Camera Top Left
                612.0, 33.0,                    // Camera Top Right
                56.0, 448.0,                    // Camera Bottom Left
                581.0, 430.5),                  // Camera Bottom Right
            new TrcHomographyMapper.Rectangle(
                -19.0, 37.5,                    // World Top Left
                24.0, 37.5,                     // World Top Right
                -4.75, 9.0,                     // World Bottom Left
                6.25, 9.0));                    // World Bottom Right
    // Limelight camera properties
    public static final int NUM_LIMELIGHT_PIPELINES = 2;
    public static final TrcVision.CameraInfo limelightParams = new TrcVision.CameraInfo()
        .setCameraInfo("Limelight3a", 640, 480)
        .setCameraFOV(54.505, 42.239)
        .setCameraPose(0.0, 0.0, 16.361, 0.0, 18.0, 0.0);

    // IntoTheDeep Robot
    public static final TrcVision.CameraInfo sampleCamParams = new TrcVision.CameraInfo()
        .setCameraInfo("Webcam 1", 640, 480)
        .setCameraPose(-4.25, 5.5, 10.608, -2.0, -32.346629699, 0.0)
        .setLensProperties(logitechC920At640x480)   // TODO: Need to calibrate camera for 320x480 for SolvePnp
        .setHomographyParams(
            new TrcHomographyMapper.Rectangle(
                14.0, 28.0,                     // Camera Top Left
                612.0, 33.0,                    // Camera Top Right
                56.0, 448.0,                    // Camera Bottom Left
                581.0, 430.5),                  // Camera Bottom Right
            new TrcHomographyMapper.Rectangle(
                -19.0, 37.5,                    // World Top Left
                24.0, 37.5,                     // World Top Right
                -4.75, 9.0,                     // World Bottom Left
                6.25, 9.0));                    // World Bottom Right
    public static final TrcVision.CameraInfo intoTheDeepLimelightParams = new TrcVision.CameraInfo()
        .setCameraInfo("Limelight3a", 640, 480)
        .setCameraFOV(54.5, 42.0)
        .setCameraPose(135.47*TrcUtil.INCHES_PER_MM, 2.073, 10.758, -3.438, 0.0, 0.0);

    public enum ArtifactType
    {
        None,
        Purple,
        Green,
        Unknown,
        Any
    }   //enum ArtifactType

    public enum LimelightPipelineType
    {
        APRIL_TAG(0),
        ARTIFACT(1);

        public final int value;
        LimelightPipelineType(int value)
        {
            this.value = value;
        }
    }   //enum LimelightPipelineType

    // Warning: EOCV converts camera stream to RGBA whereas Desktop OpenCV converts it to BGRA. Therefore, the correct
    // color conversion must be RGBA (or RGB) to whatever color space you want to convert.
    //
//    // YCrCb Color Space.
//    private static final TrcOpenCvColorBlobPipeline.ColorConversion colorConversion =
//        TrcOpenCvColorBlobPipeline.ColorConversion.RGBToYCrCb;
//    private static final double[] purpleThresholdsLow = {80.0, 139.0, 120.0};
//    private static final double[] purpleThresholdsHigh = {180.0, 160.0, 150.0};
//    private static final double[] greenThresholdsLow = {70.0, 40.0, 100.0};
//    private static final double[] greenThresholdsHigh = {220.0, 118.0, 145.0};
    // HSV Color Space.
    private static final TrcOpenCvColorBlobPipeline.ColorConversion colorConversion =
        TrcOpenCvColorBlobPipeline.ColorConversion.RGBToHSV;
    private static final double[] purpleThresholdsLow = {120.0, 60.0, 100.0};
    private static final double[] purpleThresholdsHigh = {170.0, 255.0, 255.0};
    private static final double[] greenThresholdsLow = {50.0, 70.0, 100.0};
    private static final double[] greenThresholdsHigh = {100.0, 255.0, 255.0};
    public static final TrcOpenCvColorBlobPipeline.FilterContourParams artifactFilterParams =
        new TrcOpenCvColorBlobPipeline.FilterContourParams()
            .setMinArea(50.0)
            .setMinPerimeter(20.0)
            .setWidthRange(10.0, 500.0)
            .setHeightRange(10.0, 500.0)
            .setSolidityRange(0.0, 100.0)
            .setVerticesRange(0.0, 1000.0)
            .setAspectRatioRange(0.5, 2.0);
    public static final TrcOpenCvColorBlobPipeline.FilterContourParams classifierBlobFilterParams =
        new TrcOpenCvColorBlobPipeline.FilterContourParams()
            .setMinArea(160.0)
            .setMinPerimeter(50.0)
            .setWidthRange(20.0, 250.0)
            .setHeightRange(10.0, 60.0)
            .setSolidityRange(0.0, 100.0)
            .setVerticesRange(0.0, 1000.0)
            .setAspectRatioRange(0.05, 20.0);
    private static final double artifactWidth = 5.0;  // inches
    private static final double artifactHeight = 5.0; // inches

    private static final int CLASSIFIER_ROI_LEFT = 0;
    private static final int CLASSIFIER_ROI_TOP = 50;
    private static final int CLASSIFIER_ROI_WIDTH = frontCamParams.camImageWidth;
    private static final int CLASSIFIER_ROI_HEIGHT = 100;
    private static final double RECT_ANGLE_THRESHOLD = 3.0;
    private static final double ONE_BALL_THRESHOLD = 3.0;
    private static final double TWO_BALL_THRESHOLD = 2 * ONE_BALL_THRESHOLD;
    private static final double THREE_BALL_THRESHOLD = 3 * ONE_BALL_THRESHOLD;
    private static final double FOUR_BALL_THRESHOLD = 4 * ONE_BALL_THRESHOLD;
    private static final double FIVE_BALL_THRESHOLD = 5 * ONE_BALL_THRESHOLD;
    private static final double SIX_BALL_THRESHOLD = 6 * ONE_BALL_THRESHOLD;
    private static final double SEVEN_BALL_THRESHOLD = 7 * ONE_BALL_THRESHOLD;
    private static final double EIGHT_BALL_THRESHOLD = 8 * ONE_BALL_THRESHOLD;
    // Create the pipeline parameters for both purple and green artifacts here so that Dashboard can access them.
    public static final TrcOpenCvColorBlobPipeline.PipelineParams artifactPipelineParams =
        new TrcOpenCvColorBlobPipeline.PipelineParams()
            .setAnnotation(false, false)
            .setColorConversion(colorConversion)
            .addColorThresholds(LEDIndicator.PURPLE_BLOB, true, purpleThresholdsLow, purpleThresholdsHigh)
            .addColorThresholds(LEDIndicator.GREEN_BLOB, true, greenThresholdsLow, greenThresholdsHigh)
            .buildColorThresholdSets()
            .setCircleDetection(10.0)
            .setCircleBlur(true, 9)
            .setFilterContourParams(true, artifactFilterParams);
    public static final TrcOpenCvColorBlobPipeline.PipelineParams classifierPipelineParams =
        new TrcOpenCvColorBlobPipeline.PipelineParams()
            .setAnnotation(true, false)
            .setRoi(CLASSIFIER_ROI_LEFT, CLASSIFIER_ROI_TOP, CLASSIFIER_ROI_WIDTH, CLASSIFIER_ROI_HEIGHT)
            .setColorConversion(colorConversion)
            .addColorThresholds(LEDIndicator.PURPLE_BLOB, true, purpleThresholdsLow, purpleThresholdsHigh)
            .addColorThresholds(LEDIndicator.GREEN_BLOB, true, greenThresholdsLow, greenThresholdsHigh)
            .buildColorThresholdSets()
            .setFilterContourParams(true, classifierBlobFilterParams);

    private final TrcDbgTrace tracer;
    private final Robot robot;
    private final WebcamName webcam1, webcam2;
    public FtcLimelightVision limelightVision;
    public FtcVisionAprilTag webcamAprilTagVision;
    private AprilTagProcessor webcamAprilTagProcessor;
    public FtcVisionEocvColorBlob artifactVision;
    private FtcEocvColorBlobProcessor artifactProcessor;
    public FtcVisionEocvColorBlob classifierVision;
    private FtcEocvColorBlobProcessor classifierProcessor;
    public FtcVision ftcVision;
    private FtcAuto.Alliance alliance = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object.
     */
    public Vision(Robot robot)
    {
        FtcOpMode opMode = FtcOpMode.getInstance();

        if (robot.robotInfo.webCam1 == null && RobotParams.Preferences.useWebCam)
        {
            throw new IllegalArgumentException("Must provide valid WebCam 1 info.");
        }

        this.tracer = new TrcDbgTrace();
        this.robot = robot;

        webcam1 = RobotParams.Preferences.useWebCam && robot.robotInfo.webCam1 != null?
            opMode.hardwareMap.get(WebcamName.class, robot.robotInfo.webCam1.camName): null;
        webcam2 = RobotParams.Preferences.useWebCam && robot.robotInfo.webCam2 != null?
            opMode.hardwareMap.get(WebcamName.class, robot.robotInfo.webCam2.camName): null;
        // LimelightVision (not a Vision Processor).
        if (RobotParams.Preferences.useLimelightVision && robot.robotInfo.limelight != null)
        {
            tracer.traceInfo(moduleName, "Starting LimelightVision...");
            limelightVision = new FtcLimelightVision(robot.robotInfo.limelight, this::getLimelightTargetGroundOffset);
            setLimelightPipeline(LimelightPipelineType.APRIL_TAG);
        }

        if (webcam1 != null || webcam2 != null)
        {
            // Creating Vision Processors for VisionPortal.
            ArrayList<VisionProcessor> visionProcessorsList = new ArrayList<>();

            if (RobotParams.Preferences.useWebcamAprilTagVision)
            {
                tracer.traceInfo(moduleName, "Starting Webcam AprilTagVision...");
                FtcVisionAprilTag.Parameters aprilTagParams = new FtcVisionAprilTag.Parameters()
                    .setDrawTagIdEnabled(true)
                    .setDrawTagOutlineEnabled(true)
                    .setDrawAxesEnabled(false)
                    .setDrawCubeProjectionEnabled(false)
                    .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES);
                webcamAprilTagVision = new FtcVisionAprilTag(aprilTagParams, AprilTagProcessor.TagFamily.TAG_36h11);
                webcamAprilTagProcessor = webcamAprilTagVision.getVisionProcessor();
                visionProcessorsList.add(webcamAprilTagProcessor);
            }

            if (robot.robotInfo.webCam1 != null)
            {
                if (RobotParams.Preferences.useArtifactVision)
                {
                    tracer.traceInfo(moduleName, "Starting Webcam ArtifactVision...");
                    TrcOpenCvColorBlobPipeline.SolvePnpParams solvePnpParams = null;
                    if (RobotParams.Preferences.useSolvePnp)
                    {
                        solvePnpParams =
                            new TrcOpenCvColorBlobPipeline.SolvePnpParams().setObjectSize(
                                artifactWidth, artifactHeight);
                        if (robot.robotInfo.webCam1.lensInfo != null)
                        {
                            solvePnpParams.setSolvePnpParams(
                                robot.robotInfo.webCam1.lensInfo, robot.robotInfo.webCam1.camPose);
                        }
                    }

                    artifactVision = new FtcVisionEocvColorBlob(
                        "ArtifactVision", artifactPipelineParams, solvePnpParams,
                        robot.robotInfo.webCam1.cameraRect, robot.robotInfo.webCam1.worldRect);
                    artifactProcessor = artifactVision.getVisionProcessor();
                    visionProcessorsList.add(artifactProcessor);
                    //                artifactProcessor.getPipeline().tracer.setTraceLevel(TrcDbgTrace.MsgLevel.DEBUG);
                }

                if (RobotParams.Preferences.useClassifierVision)
                {
                    tracer.traceInfo(moduleName, "Starting Webcam ClassifierVision...");
                    classifierVision = new FtcVisionEocvColorBlob(
                        "ClassifierVision", classifierPipelineParams, null, robot.robotInfo.webCam1.cameraRect,
                        robot.robotInfo.webCam1.worldRect);
                    classifierProcessor = classifierVision.getVisionProcessor();
                    visionProcessorsList.add(classifierProcessor);
                    //                classifierProcessor.getPipeline().tracer.setTraceLevel(TrcDbgTrace.MsgLevel.DEBUG);
                }
            }

            if (!visionProcessorsList.isEmpty())
            {
                VisionProcessor[] visionProcessors = new VisionProcessor[visionProcessorsList.size()];
                visionProcessorsList.toArray(visionProcessors);
                if (RobotParams.Preferences.useWebCam)
                {
                    // Use USB webcams.
                    ftcVision = new FtcVision(
                        webcam1, webcam2, robot.robotInfo.webCam1.camImageWidth, robot.robotInfo.webCam1.camImageHeight,
                        RobotParams.Preferences.showVisionView, RobotParams.Preferences.showVisionStat,
                        visionProcessors);
                }

                // Disable all vision until they are needed.
                for (VisionProcessor processor : visionProcessors)
                {
                    ftcVision.setProcessorEnabled(processor, false);
                }
            }
        }
        FtcDashboard.getInstance().addStatusUpdate(moduleName, this::updateStatus);
    }   //Vision

    /**
     * This method closes the vision portal and is normally called at the end of an opmode.
     */
    public void close()
    {
        if (ftcVision != null)
        {
            ftcVision.close();
        }
    }   //close

    /**
     * This method enables/disables FPS meter on the viewport.
     *
     * @param enabled specifies true to enable FPS meter, false to disable.
     */
    public void setFpsMeterEnabled(boolean enabled)
    {
        if (ftcVision != null)
        {
            ftcVision.setFpsMeterEnabled(enabled);
        }
    }   //setFpsMeterEnabled

    /**
     * This method returns the front webcam.
     *
     * @return front webcam.
     */
    public WebcamName getFrontWebcam()
    {
        return webcam1;
    }   //getFrontWebcam

    /**
     * This method returns the rear webcam.
     *
     * @return rear webcam.
     */
    public WebcamName getRearWebcam()
    {
        return webcam2;
    }   //getRearWebcam

    /**
     * This method returns the active camera if we have two webcams.
     *
     * @return active camera.
     */
    public WebcamName getActiveWebcam()
    {
        return ftcVision.getActiveWebcam();
    }   //getActiveWebcam

    /**
     * This method sets the active webcam.
     *
     * @param webcam specifies the webcam to be set as active.
     */
    public void setActiveWebcam(WebcamName webcam)
    {
        ftcVision.setActiveWebcam(webcam);
    }   //setActiveWebcam

    /**
     * This method displays the exposure settings on the dashboard. This helps tuning camera exposure.
     *
     * @param lineNum specifies the dashboard line number to display the info.
     */
    public void displayExposureSettings(int lineNum)
    {
        long[] exposureSetting = ftcVision.getExposureSetting();
        long currExposure = ftcVision.getCurrentExposure();
        int[] gainSetting = ftcVision.getGainSetting();
        int currGain = ftcVision.getCurrentGain();

        if (exposureSetting != null && gainSetting != null)
        {
            robot.dashboard.displayPrintf(
                lineNum, "Exp: %d (%d:%d), Gain: %d (%d:%d)",
                currExposure, exposureSetting[0], exposureSetting[1], currGain, gainSetting[0], gainSetting[1]);
        }
    }   //displayExposureSettings

    /**
     * This method enables/disables Limelight vision for the specified pipeline.
     *
     * @param pipelineType specifies the limelight pipeline type to be selected, ignore if disabled.
     * @param enabled specifies true to enable, false to disable.
     */
    public void setLimelightVisionEnabled(LimelightPipelineType pipelineType, boolean enabled)
    {
        if (limelightVision != null && enabled ^ limelightVision.isVisionEnabled())
        {
            if (enabled)
            {
                setLimelightPipeline(pipelineType);
            }
            limelightVision.setVisionEnabled(enabled);
            tracer.traceInfo(
                moduleName, "Pipeline %s is %s: running=%s",
                pipelineType, enabled? "enabled": "disabled", limelightVision.limelight.isRunning());
        }
    }   //setLimelightVisionEnabled

    /**
     * This method checks if Limelight vision is enabled.
     *
     * @return true if enabled, false if disabled.
     */
    public boolean isLimelightVisionEnabled()
    {
        return limelightVision != null && limelightVision.isVisionEnabled();
    }   //isLimelightVisionEnabled

    /**
     * This method sets the Limelight pipeline.
     *
     * @param pipelineType specifies the pipeline type.
     */
    public void setLimelightPipeline(LimelightPipelineType pipelineType)
    {
        if (limelightVision != null && limelightVision.isVisionEnabled())
        {
            limelightVision.setPipeline(pipelineType.value);
            limelightVision.setStatusResultType(
                pipelineType == LimelightPipelineType.APRIL_TAG? FtcLimelightVision.ResultType.Fiducial:
                pipelineType == LimelightPipelineType.ARTIFACT? FtcLimelightVision.ResultType.Python: null);
        }
    }   //setLimelightPipeline

    /**
     * This method calls Limelight vision to detect the object.
     *
     * @param resultType specifies the result type to look for.
     * @param matchIds specifies the object ID(s) to match for, null if no matching required.
     * @param robotHeading specifies robot heading in degrees for multi-tag localization, can be null if not provided.
     * @param comparator specifies the comparator to sort the array if provided, can be null if not provided.
     * @param lineNum specifies the dashboard line number to display the detected object info, -1 to disable printing.
     * @return detected Limelight object info.
     */
    public TrcVisionTargetInfo<FtcLimelightVision.DetectedObject> getLimelightDetectedObject(
        FtcLimelightVision.ResultType resultType, Object matchIds, Double robotHeading,
        Comparator<? super TrcVisionTargetInfo<FtcLimelightVision.DetectedObject>> comparator, int lineNum)
    {
        TrcVisionTargetInfo<FtcLimelightVision.DetectedObject> limelightInfo = null;

        if (limelightVision != null)
        {
            String objectName = null;
            int pipelineIndex = -1;

            limelightInfo = limelightVision.getBestDetectedTargetInfo(resultType, matchIds, robotHeading, comparator);
            if (limelightInfo != null)
            {
                pipelineIndex = limelightVision.getPipeline();
                switch (pipelineIndex)
                {
                    case 0:
                        objectName = (int) limelightInfo.detectedObj.objId == RobotParams.Game.blueGoalAprilTag[0]?
                            LEDIndicator.BLUE_APRILTAG: LEDIndicator.RED_APRILTAG;
                        if (robot.ledIndicator != null)
                        {
                            // Clear all previously set states first.
                            robot.ledIndicator.setStatusPatternOn(objectName, true);
                        }
                        break;

                    case 1:
                        objectName = (String)limelightInfo.detectedObj.objId;
                        if (robot.ledIndicator != null)
                        {
                            // Artifact states will turn itself off and it is the highest priority,
                            // so no need to clear previous states.
                            robot.ledIndicator.setStatusPatternOn(objectName, false);
                        }
                        break;

                    default:
                        break;
                }
            }

            if (lineNum != -1)
            {
                robot.dashboard.displayPrintf(
                    lineNum, "%s(pipeline=%d): %s",
                    objectName, pipelineIndex, limelightInfo != null? limelightInfo: "Not found.");
            }
        }

        return limelightInfo;
    }   //getLimelightDetectedObject

    /**
     * This method enables/disables the Vision Processor.
     *
     * @param processor specifies the vision processor to enable/disable.
     * @param enabled specifies true to enable, false to disable.
     */
    public void setVisionProcessorEnabled(VisionProcessor processor, boolean enabled)
    {
        if (processor != null)
        {
            ftcVision.setProcessorEnabled(processor, enabled);
        }
    }   //setVisionProcessorEnabled

    /**
     * This method checks if the Vision Processor is enabled.
     *
     * @param processor specifies the vision processor to enable/disable.
     * @return true if enabled, false if disabled.
     */
    public boolean isVisionProcessorEnabled(VisionProcessor processor)
    {
        return processor != null && ftcVision.isVisionProcessorEnabled(processor);
    }   //isVisionProcessorEnabled

    /**
     * This method enables/disables Webcam AprilTag vision.
     *
     * @param enabled specifies true to enable, false to disable.
     */
    public void setWebcamAprilTagVisionEnabled(boolean enabled)
    {
        setVisionProcessorEnabled(webcamAprilTagProcessor, enabled);
    }   //setWebcamAprilTagVisionEnabled

    /**
     * This method checks if Webcam AprilTag vision is enabled.
     *
     * @return true if enabled, false if disabled.
     */
    public boolean isWebcamAprilTagVisionEnabled()
    {
        return isVisionProcessorEnabled(webcamAprilTagProcessor);
    }   //isWebcamAprilTagVisionEnabled

    /**
     * This method calls Webcam AprilTag vision to detect the AprilTag object.
     *
     * @param aprilTagIds specifies an array of AprilTag ID to look for, null if match to any ID.
     * @param lineNum specifies the dashboard line number to display the detected object info, -1 to disable printing.
     * @return detected AprilTag object info.
     */
    public TrcVisionTargetInfo<FtcVisionAprilTag.DetectedObject> getWebcamDetectedAprilTag(
        int[] aprilTagIds, int lineNum)
    {
        TrcVisionTargetInfo<FtcVisionAprilTag.DetectedObject> aprilTagInfo =
            webcamAprilTagVision.getBestDetectedTargetInfo(aprilTagIds, null);

        if (aprilTagInfo != null && robot.ledIndicator != null)
        {
            // This is assuming vision is looking for either 20 or 24 and not the obelisk.
            robot.ledIndicator.setStatusPatternOn(
                aprilTagInfo.detectedObj.aprilTagDetection.id == RobotParams.Game.blueGoalAprilTag[0] ?
                    LEDIndicator.BLUE_APRILTAG : LEDIndicator.RED_APRILTAG, true);
        }

        if (lineNum != -1)
        {
            robot.dashboard.displayPrintf(
                lineNum, "AprilTag[%s]: %s",
                Arrays.toString(aprilTagIds), aprilTagInfo != null ? aprilTagInfo : "Not found.");
        }

        return aprilTagInfo;
    }   //getWebcamDetectedAprilTag

    /**
     * This method calculates the robot's absolute field location with the detected AprilTagInfo.
     *
     * @param aprilTagInfo specifies the detected AprilTag info.
     * @return robot field location.
     */
    public TrcPose2D getRobotFieldPose(TrcVisionTargetInfo<FtcVisionAprilTag.DetectedObject> aprilTagInfo)
    {
        TrcPose2D robotPose = null;

        if (aprilTagInfo != null)
        {
            TrcPose2D aprilTagFieldPose =
                RobotParams.Game.APRILTAG_POSES[aprilTagInfo.detectedObj.aprilTagDetection.id - 1];
            TrcPose2D camPoseOnBot = new TrcPose2D(
                robot.robotInfo.webCam1.camPose.x, robot.robotInfo.webCam1.camPose.y,
                robot.robotInfo.webCam1.camPose.yaw);
            robotPose = aprilTagFieldPose.addRelativePose(aprilTagInfo.objPose.invert())
                                         .addRelativePose(camPoseOnBot.invert());
            tracer.traceInfo(
                moduleName,
                "AprilTagId=" + aprilTagInfo.detectedObj.aprilTagDetection.id +
                ", aprilTagFieldPose=" + aprilTagFieldPose +
                ", aprilTagPoseFromCamera=" + aprilTagInfo.objPose +
                ", cameraPose=" + camPoseOnBot +
                ", robotPose=%s" + robotPose);
        }

        return robotPose;
    }   //getRobotFieldPose

    /**
     * This method determines the target depth and bearing by using AprilTag Vision.
     *
     * @param aprilTagInfo specifies the detected AprilTag info.
     * @return array of two doubles, first of which is target depth and second of which is target bearing.
     */
    public double[] getAimInfoByVision(TrcVisionTargetInfo<FtcLimelightVision.DetectedObject> aprilTagInfo)
    {
        int aprilTagId = (int) aprilTagInfo.detectedObj.objId;
        TrcPose2D targetPose = aprilTagInfo.objPose.addRelativePose(
            aprilTagId == 20?
                RobotParams.Game.BLUE_APRILTAG_TO_CORNER: RobotParams.Game.RED_APRILTAG_TO_CORNER);
        double targetDepth = aprilTagInfo.objDepth;
        // targetPose is in camera space and the camera is mounted on a turret. Adjust angle to robot space by adding
        // turret angle.
        double targetBearing = targetPose.angle + robot.shooter.getPanAngle();
        tracer.traceDebug(
            moduleName, "aprilTagPose{%d}=%s, targetPose=%s, depth=%f, bearing=%f",
            aprilTagId, aprilTagInfo.objPose, targetPose, targetDepth, targetBearing, targetBearing);

        return new double[] {targetDepth, targetBearing};
    }   //getAimInfoByVision

    /**
     * This method determines the target depth and bearing by using Odometry.
     *
     * @param alliance specifies the alliance goal to shoot at.
     * @return array of two doubles, first of which is target depth and second of which is target bearing.
     */
    public double[] getAimInfoByOdometry(FtcAuto.Alliance alliance)
    {
        TrcPose2D robotPose = robot.robotBase.driveBase.getFieldPosition();
        TrcPose2D goalFieldPose = alliance == FtcAuto.Alliance.BLUE_ALLIANCE?
            RobotParams.Game.BLUE_CORNER_POSE: RobotParams.Game.RED_CORNER_POSE;
        TrcPose2D targetPose = goalFieldPose.relativeTo(robotPose);
        // TODO: Fix this code, it doesn't work.
        TrcPose2D aprilTagToCornerPose =
            alliance == FtcAuto.Alliance.BLUE_ALLIANCE?
                RobotParams.Game.BLUE_APRILTAG_TO_CORNER: RobotParams.Game.RED_APRILTAG_TO_CORNER;
        TrcPose2D aprilTagPose = targetPose.addRelativePose(aprilTagToCornerPose.invert());
        double targetDepth = TrcUtil.magnitude(aprilTagPose.x, aprilTagPose.y);
        double targetBearing = targetPose.angle;
        tracer.traceDebug(
            moduleName, "robotPose=%s, targetPose=%s, aprilTagPose=%s, depth=%f, bearing=%f",
            robotPose, targetPose, aprilTagPose, targetDepth, targetBearing);

        return new double[] {targetDepth, targetBearing};
    }   //getAimInfoByOdometry

    /**
     * This method uses vision to find an AprilTag and uses the AprilTag's absolute field location and its relative
     * position from the camera to calculate the robot's absolute field location.
     *
     * @param robotHeading specifies robot heading in degrees for multi-tag localization, can be null if not provided.
     * @return robot field location.
     */
    public TrcPose2D getRobotFieldPose(Double robotHeading)
    {
        TrcPose2D robotPose = null;

        if (isLimelightVisionEnabled())
        {
            TrcVisionTargetInfo<FtcLimelightVision.DetectedObject> aprilTagInfo =
                getLimelightDetectedObject(
                    FtcLimelightVision.ResultType.Fiducial, RobotParams.Game.anyGoalAprilTags, robotHeading, null, -1);

            if (aprilTagInfo != null)
            {
                robotPose = aprilTagInfo.detectedObj.robotPose;
                tracer.traceInfo(moduleName, "getRobotFieldPose=%s (obj=%s)", robotPose, aprilTagInfo.detectedObj);
            }
        }
        else if (isWebcamAprilTagVisionEnabled())
        {
            // Find any AprilTag in view.
            TrcVisionTargetInfo<FtcVisionAprilTag.DetectedObject> aprilTagInfo = getWebcamDetectedAprilTag(
                RobotParams.Game.anyGoalAprilTags, -1);

            if (aprilTagInfo != null)
            {
                robotPose = getRobotFieldPose(aprilTagInfo);
            }
        }

        return robotPose;
    }   //getRobotFieldPose

    /**
     * This method uses vision to find an AprilTag and uses the AprilTag's absolute field location and its relative
     * position from the camera to calculate the robot's absolute field location.
     *
     * @return robot field location.
     */
    public TrcPose2D getRobotFieldPose()
    {
        return getRobotFieldPose((Double) null);
    }   //getRobotFieldPose

    /**
     * This method enables/disables the Dashboard Streaming of the specified ColorBlob processor.
     *
     * @param processor specifies the ColorBlob processor to have the Dashboard stream enabled/disabled.
     * @param enabled specifies true to enable stream, false to disable.
     */
    public void setDashboardStreamEnabled(FtcEocvColorBlobProcessor processor, boolean enabled)
    {
        if (RobotParams.Preferences.streamWebcamToDashboard)
        {
            if (enabled)
            {
                processor.enableDashboardStream();
            }
            else
            {
                processor.disableDashboardStream();
            }
        }
    }   //setDashboardStreamEnabled

    /**
     * This method enables/disables vision for the specified artifact type.
     *
     * @param artifactType specifies the artifact type to be detected.
     * @param enabled specifies true to enable, false to disable.
     */
    public void setArtifactVisionEnabled(ArtifactType artifactType, boolean enabled)
    {
        TrcOpenCvColorBlobPipeline artifactPipeline =
            artifactProcessor != null? artifactProcessor.getPipeline(): null;

        if (artifactPipeline != null)
        {
            switch (artifactType)
            {
                case Purple:
                    artifactPipeline.setColorThresholdsEnabled(LEDIndicator.PURPLE_BLOB, enabled);
                    if (enabled)
                    {
                        artifactPipeline.setColorThresholdsEnabled(LEDIndicator.GREEN_BLOB, false);
                    }
                    break;

                case Green:
                    artifactPipeline.setColorThresholdsEnabled(LEDIndicator.GREEN_BLOB, enabled);
                    if (enabled)
                    {
                        artifactPipeline.setColorThresholdsEnabled(LEDIndicator.PURPLE_BLOB, false);
                    }
                    break;

                case Any:
                    artifactPipeline.setColorThresholdsEnabled(LEDIndicator.PURPLE_BLOB, enabled);
                    artifactPipeline.setColorThresholdsEnabled(LEDIndicator.GREEN_BLOB, enabled);
                    break;
            }

            if (enabled)
            {
                // Start Dashboard Stream before turning on Artifact Processor.
                setDashboardStreamEnabled(artifactProcessor, true);
                setVisionProcessorEnabled(artifactProcessor, true);
            }
            else
            {
                // We are disabling a color threshold set in the Artifact pipeline. If all color threshold sets
                // are disabled, disable the Artifact vision processor as well.
                if (!artifactPipeline.isColorThresholdsEnabled(LEDIndicator.PURPLE_BLOB) &&
                    !artifactPipeline.isColorThresholdsEnabled(LEDIndicator.GREEN_BLOB))
                {
                    setVisionProcessorEnabled(artifactProcessor, false);
                    setDashboardStreamEnabled(artifactProcessor, false);
                }
            }
        }
    }   //setArtifactVisionEnabled

    /**
     * This method checks if vision is enabled for the specified color artifact type.
     *
     * @param artifactType specifies the artifact type to be detected.
     * @return true if enabled, false if disabled.
     */
    public boolean isArtifactVisionEnabled(ArtifactType artifactType)
    {
        boolean enabled = false;
        TrcOpenCvColorBlobPipeline artifactPipeline =
            artifactProcessor != null && isVisionProcessorEnabled(artifactProcessor)?
                artifactProcessor.getPipeline(): null;

        if (artifactPipeline != null)
        {
            switch (artifactType)
            {
                case Purple:
                    enabled = artifactPipeline.isColorThresholdsEnabled(LEDIndicator.PURPLE_BLOB);
                    break;

                case Green:
                    enabled = artifactPipeline.isColorThresholdsEnabled(LEDIndicator.GREEN_BLOB);
                    break;

                case Any:
                    enabled = artifactPipeline.isColorThresholdsEnabled(LEDIndicator.PURPLE_BLOB) ||
                              artifactPipeline.isColorThresholdsEnabled(LEDIndicator.GREEN_BLOB);
                    break;
            }
        }

        return enabled;
    }   //isArtifactVisionEnabled

    /**
     * This method calls Artifact vision to detect the specified artifact.
     *
     * @param artifactType specifies the artifact type to be detected.
     * @param groundOffset specifies the ground offset of the detected sample.
     * @param lineNum specifies the dashboard line number to display the detected object info, -1 to disable printing.
     * @return detected artifact object info.
     */
    public TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject> getDetectedArtifact(
        ArtifactType artifactType, double groundOffset, int lineNum)
    {
        TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject> artifactInfo = null;

        if (isArtifactVisionEnabled(artifactType))
        {
            artifactInfo = artifactVision == null? null:
                artifactVision.getBestDetectedTargetInfo(
                    this::artifactFilter, artifactType, this::compareDistanceY, groundOffset,
                    robot.robotInfo.webCam1.camPose.z);
        }

        if (artifactInfo != null && robot.ledIndicator != null)
        {
            // Artifact state will turn itself off and is the highest priority.
            robot.ledIndicator.setStatusPatternOn(artifactInfo.detectedObj.label, false);
        }

        if (lineNum != -1)
        {
            if (artifactInfo != null)
            {
                robot.dashboard.displayPrintf(lineNum, "%s: %s", artifactInfo.detectedObj.label, artifactInfo);
            }
            else
            {
                robot.dashboard.displayPrintf(lineNum, "No Artifact found.");
            }
        }

        return artifactInfo;
    }   //getDetectedArtifact

    /**
     * This method enables/disables Classifier vision.
     *
     * @param enabled specifies true to enable, false to disable.
     */
    public void setClassifierVisionEnabled(boolean enabled)
    {
        TrcOpenCvColorBlobPipeline classifierPipeline =
            classifierProcessor != null? classifierProcessor.getPipeline(): null;

        if (classifierPipeline != null)
        {
            if (enabled)
            {
                // Enable both color threshold sets for Classifier Vision in case they are not already enabled.
                classifierPipeline.setColorThresholdsEnabled(LEDIndicator.PURPLE_BLOB, true);
                classifierPipeline.setColorThresholdsEnabled(LEDIndicator.GREEN_BLOB, true);
                // Start Dashboard Stream before turning on Classifier Processor.
                setDashboardStreamEnabled(classifierProcessor, true);
                setVisionProcessorEnabled(classifierProcessor, true);
            }
            else
            {
                // We are disabling Classifier Vision, just disable the Classifier processor but no need to disable
                // the color threshold set.
                setVisionProcessorEnabled(classifierProcessor, false);
                setDashboardStreamEnabled(classifierProcessor, false);
            }
        }
    }   //setClassifierVisionEnabled

    /**
     * This method checks if Classifier Vision is enabled.
     *
     * @return true if enabled, false if disabled.
     */
    public boolean isClassifierVisionEnabled()
    {
        return classifierProcessor != null && isVisionProcessorEnabled(classifierProcessor);
    }   //isClassifierVisionEnabled

    /**
     * The method uses vision to detect all Artifacts in the classifier and returns an array of 9 slots specifying the
     * type of artifacts in each slot. It assumes Classifier pipeline is enabled to detect Any artifacts.
     *
     * @param alliance specifies the alliance color for sorting the array.
     */
    public ArtifactType[] getClassifierArtifacts(FtcAuto.Alliance alliance)
    {
        ArtifactType[] artifacts = null;

        if (isClassifierVisionEnabled())
        {
            // compareDistanceX is using alliance to determine the sort order of color blobs in the classifier.
            this.alliance = alliance;
            ArrayList<TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject>> blobs =
                classifierVision.getDetectedTargetsInfo(null, null, this::compareDistanceX, 0.0, 0.0);

            if (blobs != null)
            {
                int index = 0;
                artifacts = new ArtifactType[9];

                for (int i = 0; i < blobs.size(); i++)
                {
                    TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject> blob = blobs.get(i);
                    ArtifactType artifactType =
                        blob.detectedObj.label.equals(LEDIndicator.PURPLE_BLOB)?
                            ArtifactType.Purple : ArtifactType.Green;
                    int count = getArtifactCount(blob);
                    tracer.traceDebug(moduleName, "[%d] %d %s", i, count, artifactType);
                    robot.dashboard.putString(
                        "Blob" + i,
                        String.format(
                            Locale.US, "%d %s(%.1f/%.1f=%.1f, angle=%.1f)",
                            count, artifactType, blob.objPixelWidth, blob.objPixelHeight, getAspectRatio(blob),
                            blob.objRotatedRectAngle));

                    for (int j = 0; j < count; j++)
                    {
                        if (index < artifacts.length)
                        {
                            artifacts[index++] = artifactType;
                        }
                        else
                        {
                            tracer.traceWarn(
                                moduleName, "Number artifact exceeds capacity (artifact=%s, count=%d, blob=%s)",
                                artifactType, count, blob);
                            break;
                        }
                    }
                }

                for (int k = blobs.size(); k < artifacts.length; k++)
                {
                    robot.dashboard.putString("Blob" + k, "");
                }

                for (int k = index; k < artifacts.length; k++)
                {
                    artifacts[k] = ArtifactType.None;
                }

                robot.dashboard.putString("Classifier", Arrays.toString(artifacts));
            }
        }

        return artifacts;
    }   //getClassifierArtifacts

    /**
     * This method calls Classifier Vision to determine motif sequence to shoot next. It assumes Classifier Vision
     * is enabled.
     *
     * @param alliance specifies the alliance color.
     * @param obeliskMotif specifies the obelisk motif pattern.
     * @param useVision specifies true to use Classifier Vision, false otherwise.
     * @return the motif sequence for shooting, null if data not ready or not detecting classifier artifacts.
     */
    public ArtifactType[] getMotifSequence(FtcAuto.Alliance alliance, ArtifactType[] obeliskMotif, boolean useVision)
    {
        ArtifactType[] motifSequence = null;

        if (useVision)
        {
            ArtifactType[] classifierArtifacts = getClassifierArtifacts(alliance);

            if (classifierArtifacts != null)
            {
                tracer.traceInfo(moduleName, "***** ClassifierArtifacts=" + Arrays.toString(classifierArtifacts));
                int noneIndex = -1;
                for (int i = 0; i < classifierArtifacts.length; i++)
                {
                    if (classifierArtifacts[i] == Vision.ArtifactType.None)
                    {
                        noneIndex = i%obeliskMotif.length;
                        tracer.traceInfo(moduleName, "***** First classifier empty slot=" + noneIndex);
                        break;
                    }
                }

                if (noneIndex != -1)
                {
                    motifSequence = new Vision.ArtifactType[3];
                    for (int i = 0; i < motifSequence.length; i++)
                    {
                        motifSequence[i] = obeliskMotif[noneIndex];
                        noneIndex = (noneIndex + 1)%obeliskMotif.length;
                    }
                }
            }
        }
        else
        {
            motifSequence = obeliskMotif.clone();
        }

        return motifSequence;
    }   //getMotifSequence

    /**
     * This method calculates the aspect ratio of the detected blob.
     *
     * @param blob specifies the detected blob.
     * @return calculated the aspect ratio, or NaN if cannot be determined.
     */
    private double getAspectRatio(TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject> blob)
    {
        double aspectRatio =
            blob.objPixelWidth != null && blob.objPixelHeight != null && blob.objPixelHeight != 0.0?
            (double) blob.objPixelWidth / (double) blob.objPixelHeight: Double.NaN;
        tracer.traceDebug(
            moduleName, "%.1f/%.1f=%.1f, obj=%s", blob.objPixelWidth, blob.objPixelHeight, aspectRatio, blob);
        return aspectRatio;
    }   //getAspectRatio

    /**
     * This method checks the detected blob aspect ratio to determine how many artifacts are in the detected blob.
     *
     * @param blob specifies the detected blob.
     * @return count of artifact in the detected object.
     */
    private int getArtifactCount(TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject> blob)
    {
        double blobRectAngle = Math.abs(blob.objRotatedRectAngle) % 180.0;
        if (blobRectAngle > 90.0) blobRectAngle = Math.abs(180.0 - blobRectAngle);
        boolean firstSingleBall = blobRectAngle < RECT_ANGLE_THRESHOLD;
        double aspectRatio = getAspectRatio(blob);
        return firstSingleBall || aspectRatio <= ONE_BALL_THRESHOLD? 1:
               aspectRatio <= TWO_BALL_THRESHOLD ? 2 :
               aspectRatio <= THREE_BALL_THRESHOLD ? 3 :
               aspectRatio <= FOUR_BALL_THRESHOLD ? 4 :
               aspectRatio <= FIVE_BALL_THRESHOLD ? 5 :
               aspectRatio <= SIX_BALL_THRESHOLD ? 6 :
               aspectRatio <= SEVEN_BALL_THRESHOLD ? 8 :
               aspectRatio <= EIGHT_BALL_THRESHOLD ? 8 : 9;
    }   //getArtifactCount

    /**
     * This method returns the Limelight target Z offset from ground.
     *
     * @param resultType specifies the detected object result type.
     * @return target ground offset.
     */
    private double getLimelightTargetGroundOffset(FtcLimelightVision.ResultType resultType)
    {
        double offset;

        switch (resultType)
        {
            case Fiducial:
                offset = 29.5;
                break;

            case Python:
            default:
                offset = 0.0;
                break;
        }

        return offset;
    }   //getLimelightTargetGroundOffset

    /**
     * This method is called by Vision to validate if the detected artifact matches expectation for filtering.
     *
     * @param artifactInfo specifies the detected artifact info.
     * @param context specifies the expected color artifact type.
     * @return true if it matches expectation, false otherwise.
     */
    public boolean artifactFilter(
        TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject> artifactInfo, Object context)
    {
        ArtifactType artifactType = (ArtifactType) context;
        boolean match = false;

        switch (artifactType)
        {
            case Purple:
                match = artifactInfo.detectedObj.label.equals(LEDIndicator.PURPLE_BLOB);
                break;

            case Green:
                match = artifactInfo.detectedObj.label.equals(LEDIndicator.GREEN_BLOB);
                break;

            case Any:
                match = artifactInfo.detectedObj.label.equals(LEDIndicator.PURPLE_BLOB) ||
                        artifactInfo.detectedObj.label.equals(LEDIndicator.GREEN_BLOB);
                break;
        }

        return match;
    }   //artifactFilter

//    /**
//     * This method is called by Vision to validate if the detected color blob is in the classifier by checking its
//     * vertical position is in the ROI of the classifier.
//     *
//     * @param blobInfo specifies the detected blob info.
//     * @param context not used.
//     * @return true if it matches expectation, false otherwise.
//     */
//    public boolean classifierBlobFilter(
//        TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject> blobInfo, Object context)
//    {
//        return (blobInfo.detectedObj.label.equals(LEDIndicator.PURPLE_BLOB) ||
//                blobInfo.detectedObj.label.equals(LEDIndicator.GREEN_BLOB)) &&
//               blobInfo.objRect.y >= CLASSIFIER_HEIGHT_THRESHOLD_LOW &&
//               blobInfo.objRect.y <= CLASSIFIER_HEIGHT_THRESHOLD_HIGH;
//    }   //classifierBlobFilter

    /**
     * This method is called by the Arrays.sort to sort the target object by increasing distance X. The sort direction
     * will be determined by the alliance color.
     *
     * @param a specifies the first target
     * @param b specifies the second target.
     * @return negative value if a has closer distance than b, 0 if a and b have equal distances, positive value
     *         if a has higher distance than b.
     */
    public int compareDistanceX(
        TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject> a,
        TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject> b)
    {
        int diff = (int)((b.objPose.x - a.objPose.x)*100);
        return alliance == FtcAuto.Alliance.RED_ALLIANCE? diff: -diff;
    }   //compareDistanceX

    /**
     * This method is called by the Arrays.sort to sort the target object by increasing distance Y.
     *
     * @param a specifies the first target
     * @param b specifies the second target.
     * @return negative value if a has closer distance than b, 0 if a and b have equal distances, positive value
     *         if a has higher distance than b.
     */
    public int compareDistanceY(
        TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject> a,
        TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject> b)
    {
        return (int)((b.objPose.y - a.objPose.y)*100);
    }   //compareDistanceY

    /**
     * This method update the dashboard with vision status.
     *
     * @param lineNum specifies the starting line number to print the subsystem status.
     * @param slowLoop specifies true if this is a slow loop, false otherwise.
     * @return updated line number for the next subsystem to print.
     */
    public int updateStatus(int lineNum, boolean slowLoop)
    {
        if (RobotParams.Preferences.showVisionStatus)
        {
            if (slowLoop)
            {
                if (limelightVision != null)
                {
                    lineNum = limelightVision.updateStatus(lineNum);
                }

                if (webcamAprilTagVision != null)
                {
                    lineNum = webcamAprilTagVision.updateStatus(lineNum);
                }

                if (artifactVision != null)
                {
                    lineNum = artifactVision.updateStatus(lineNum);
                }

                if (classifierVision != null)
                {
                    lineNum = classifierVision.updateStatus(lineNum);
                }
            }
        }

        return lineNum;
    }   //updateStatus

}   //class Vision
