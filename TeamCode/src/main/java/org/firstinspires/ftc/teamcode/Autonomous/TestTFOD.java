package org.firstinspires.ftc.teamcode.Autonomous;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

/*
 * This OpMode illustrates the basics of TensorFlow Object Detection,
 * including Java Builder structures for specifying Vision parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@Autonomous(name = "TestTFOD", group = "Concept", preselectTeleOp = "Payload_TeleOp")
public class TestTFOD extends LinearOpMode {
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "RedTeamProp.tflite";
    private AprilTagProcessor aprilTag;
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
//    private static final String TFO
//    TFOD_MODEL_FILE = "sdcard/FIRST/tflitemodels/Red_Team_Prop.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "Red Team Prop",
    };
    boolean LEFT;
    boolean RIGHT;
    boolean CENTER;
    boolean Seen;

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;
    private DcMotor fl, fr, bl, br;
    private CRServo serIn1, serIn2;
    private Servo serIn4, serIn5;
    public void moveForward(double power) {
        fl.setPower(-power);
        fr.setPower(-power);
        bl.setPower(-power);
        br.setPower(-power);
    }
    public void moveBackward(double power) {
        fl.setPower(power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(power);
    }
    public void turnLeft(double power) {
        fl.setPower(power);
        fr.setPower(-power);
        bl.setPower(power);
        br.setPower(-power);
    }
    public void turnRight(double power) {
        fl.setPower(-power);
        fr.setPower(power);
        bl.setPower(-power);
        br.setPower(power);
    }
    public void strafeRight(double power) {
        fl.setPower(power);
        fr.setPower(-power);
        bl.setPower(-power);
        br.setPower(power);
    }
    public void strafeLeft(double power) {
        fl.setPower(-power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(-power);
    }
    public void Idle() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }

    @Override
    public void runOpMode() throws InterruptedException{
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");
        serIn1 = hardwareMap.get(CRServo.class, "serIn1");
        serIn2 = hardwareMap.get(CRServo.class, "serIn2");
        serIn4 = hardwareMap.get(Servo.class, "serIn4");
        serIn5 = hardwareMap.get(Servo.class, "serIn5");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        serIn1.setDirection(CRServo.Direction.REVERSE);
        serIn2.setDirection(CRServo.Direction.REVERSE);
        serIn5.setDirection(Servo.Direction.REVERSE);
        serIn4.setDirection(Servo.Direction.FORWARD);

        serIn4.setPosition(0);
        serIn5.setPosition(0);

        initVisionPortal(tfod);
        Seen = false;
        CENTER = false;
        RIGHT = false;
        LEFT = false;
        boolean placed = false;
        int step2 = 1;
        boolean exe = false;
        boolean exe2 = false;
        boolean dontre = false;
        boolean tagFound = false;
        int tagOfInterest = 5;
        int step = 2;
        boolean desYaw = false;
        boolean desX = false;
        boolean desZ = false;
        boolean placedPixel = false;
        boolean checkYaw = false;
        boolean checkYaw2 = false;
        boolean parkNeeded = false;
        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                telemetryTfod();
                if (step2 == 1) {
                    moveForward(0.5);
                    TimeUnit.MILLISECONDS.sleep(120);
                    Idle();
                    TimeUnit.MILLISECONDS.sleep(100);
                    step2++;
                }
                if (step2 == 2 && !LEFT && !RIGHT) {
                    moveForward(0.5);
                    TimeUnit.MILLISECONDS.sleep(100);
                    Idle();
                    step2++;
                }
//                if (!exe2 && !LEFT && !CENTER){
//                    strafeRight(0.5);
//                    TimeUnit.MILLISECONDS.sleep(100);
//                    Idle();
//                    exe2 = true;
//                }
                // Push telemetry to the Driver Station.
                telemetry.update();
                //TODO: Insert conditions
                if (!dontre) {
                    if (Seen && !placed) {
                        if (CENTER) {
                            telemetry.addLine("Center " + CENTER);
                            telemetry.update();
                            moveForward(0.5);
                            TimeUnit.MILLISECONDS.sleep(1000);
                            Idle();
                            step2++;
                            placed = true;
                        } else if (RIGHT) {
                            telemetry.addLine("Right " + RIGHT);
                            telemetry.update();
                            TimeUnit.MILLISECONDS.sleep(550);
                            turnRight(0.5);
                            TimeUnit.MILLISECONDS.sleep(250);
                            Idle();
                            moveForward(0.5);
                            TimeUnit.MILLISECONDS.sleep(850);
                            Idle();
                            step2++;
                            placed = true;
                        } else if (LEFT) {
                            telemetry.addLine("Left " + LEFT);
                            telemetry.update();
                            moveForward(0.5);
                            TimeUnit.MILLISECONDS.sleep(250);
                            Idle();
                            TimeUnit.MILLISECONDS.sleep(550);
                            turnLeft(0.5);
                            TimeUnit.MILLISECONDS.sleep(450);
                            Idle();
                            moveForward(0.5);
                            TimeUnit.MILLISECONDS.sleep(340);
                            Idle();
                            step2++;
                            placed = true;
                        }
                    } else {
                        Idle();
                    }
                    if (placed && !exe) {
                        if (CENTER && !exe) {
                            moveBackward(0.5);
                            TimeUnit.MILLISECONDS.sleep(300);
                            Idle();
                            turnRight(0.5);
                            TimeUnit.MILLISECONDS.sleep(980);
                            Idle();
                            exe = true;
                            visionPortal.close();
                            initVisionPortal(aprilTag);
                            dontre = true;
                        }
                        if (RIGHT && !exe) {
                            moveBackward(0.5);
                            TimeUnit.MILLISECONDS.sleep(500);
                            Idle();
                            turnRight(0.5);
                            TimeUnit.MILLISECONDS.sleep(730);
                            Idle();
                            strafeRight(0.5);
                            TimeUnit.MILLISECONDS.sleep(200);
                            Idle();
                            step2++;
                            exe = true;
                            visionPortal.close();
                            initVisionPortal(aprilTag);
                            dontre = true;
                        }
                        if (LEFT && !exe) {
                            moveBackward(0.5);
                            TimeUnit.MILLISECONDS.sleep(500);
                            Idle();
                            turnRight(0.5);
                            TimeUnit.MILLISECONDS.sleep(1200);
                            Idle();
                            strafeLeft(0.5);
                            TimeUnit.MILLISECONDS.sleep(200);
                            Idle();
                            step2++;
                            exe = true;
                            visionPortal.close();
                            initVisionPortal(aprilTag);
                            dontre = true;
                        }
                    }
                }
                if (dontre) {
                ArrayList<AprilTagDetection> currentDetections = aprilTag.getDetections();
                telemetry.addData("# AprilTags Detected", currentDetections.size());

                // Step through the list of detections and display info for each one.
                for (AprilTagDetection detection : currentDetections) {
                    if (detection.metadata != null) {
                        if (detection.id == tagOfInterest) {
                            tagFound = true;
                            break;
                        }
                    }   // end for() loop

                    // Add "key" information to telemetry
                    telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
                    telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
                    telemetry.addLine("RBE = Range, Bearing & Elevation");
                    telemetry.update();
                    //Share the CPU.
                    sleep(20);
                    if (tagFound) {
                        double deadzone = 0.1;

                        telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                        do {
                            if (step == 2 && !desYaw && opModeIsActive()) {
                                telemetry.addData("step: ", step);
                                telemetry.update();
                                if (detection.ftcPose.yaw < -3) {
                                    turnLeft(0.2);
                                } else if (detection.ftcPose.yaw > 3) {
                                    turnRight(0.2);
                                } else {
                                    Idle();
                                    desYaw = true;
                                    telemetry.addData("desYaw: ", desYaw);
                                    telemetry.update();
                                    step++;
                                }
                            }
                            if (step == 3 && !desX && opModeIsActive()) {
                                telemetry.addData("step: ", step);
                                telemetry.update();
                                if (detection.ftcPose.x < -deadzone) {
                                    strafeLeft(0.4);
                                } else if (detection.ftcPose.x > deadzone) {
                                    // do something else
                                    strafeRight(0.4);
                                } else {
                                    Idle();
                                    desX = true;
                                    telemetry.addData("desX: ", desX);
                                    telemetry.update();
                                    step++;
                                }
                            }
                            if (step == 4 && !checkYaw && opModeIsActive()) {
                                telemetry.addData("step: ", step);
                                telemetry.update();
                                if (detection.ftcPose.yaw < -5) {
                                    turnLeft(0.25);
                                } else if (detection.ftcPose.yaw > 5) {
                                    turnRight(0.25);
                                } else {
                                    Idle();
                                    checkYaw = true;
                                    telemetry.addData("checkYaw: ", checkYaw);
                                    telemetry.update();
                                    step++;
                                }
                            }
                            if (step == 5 && !desZ && opModeIsActive()) {
                                telemetry.addData("step: ", step);
                                telemetry.update();
                                if (detection.ftcPose.z > 0.90) {
                                    moveForward(0.20);
                                } else {
                                    Idle();
                                    desZ = true;
                                    telemetry.addData("desZ: ", desZ);
                                    telemetry.update();
                                    step++;
                                }
                            }
                            if (step == 6 && !checkYaw2 && opModeIsActive()) {
                                telemetry.addData("step: ", step);
                                telemetry.update();
                                if (detection.ftcPose.yaw < -2) {
                                    turnLeft(0.1);
                                } else if (detection.ftcPose.yaw > 2) {
                                    turnRight(0.1);
                                } else {
                                    Idle();
                                    checkYaw2 = true;
                                    telemetry.addData("checkYaw2: ", checkYaw2);
                                    telemetry.update();
                                    step++;
                                }
                            }
                            if (desX && desZ) {
                                telemetry.addLine("Reached the april tag.");
                                telemetry.addLine("Placing the pixel");
                                telemetry.update();
                                serIn4.setPosition(0.75);
                                serIn5.setPosition(0.75);
                                TimeUnit.SECONDS.sleep(1);
                                placedPixel = true;
                            }
                            if (placedPixel) {
                                serIn1.setPower(0.5);
                                serIn2.setPower(-0.5);
                                TimeUnit.MILLISECONDS.sleep(500);
                                serIn1.setPower(0);
                                serIn2.setPower(0);
                                TimeUnit.SECONDS.sleep(1);
                                serIn4.setPosition(0);
                                serIn5.setPosition(0);
                                step++;
                                moveBackward(0.3);
                                TimeUnit.MILLISECONDS.sleep(300);
                                Idle();
                                TimeUnit.MILLISECONDS.sleep(300);
                                turnRight(0.5);
                                TimeUnit.MILLISECONDS.sleep(845);
                                Idle();
                                moveForward(0.5);
                            /*Later, this distance needs to be altered
                            because the distance between the april tags
                            makes the distance to park longer or shorter.
                             */
                                TimeUnit.MILLISECONDS.sleep(820);
                                Idle();
                                turnLeft(0.5);
                                TimeUnit.MILLISECONDS.sleep(700);
                                Idle();
                            /*
                            moveForward(0.5);
                            TimeUnit.MILLISECONDS.sleep(500);
                            */
                                Idle();
                                telemetry.addLine("-------------------------------");
                                telemetry.addLine("Autonomous complete.");
                                telemetry.addLine("-------------------------------");
                                telemetry.update();
                            }

                        } while (step == 9 && desX && checkYaw && desZ && checkYaw2 && opModeIsActive() && parkNeeded);
                    /*
                        if (step == 66) {
                            strafeRight(0.5);
                            TimeUnit.MILLISECONDS.sleep(250);
                            Idle();
                            telemetry.addLine("Strafed Left");
                            telemetry.addLine("Ready to park.");
                            telemetry.update();
                            moveForward(0.5);
                            TimeUnit.MILLISECONDS.sleep(250);
                            Idle();
                            telemetry.addLine("Parked");
                            telemetry.update();
                            parkNeeded = false;
                        } else if (!parkNeeded) {
                            step++;
                        }
                        */

                    }
                    }
                }
            }
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }   // end runOpMode()

    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initVisionPortal(VisionProcessor processor) {

        // Create the TensorFlow processor by using a builder.
        if (processor == tfod) {
            tfod = new TfodProcessor.Builder()

                    // With the following lines commented out, the default TfodProcessor Builder
                    // will load the default model for the season. To define a custom model to load,
                    // choose one of the following:
                    //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                    //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
//                .setModelFileName(TFOD_MODEL_FILE)
                    .setModelAssetName(TFOD_MODEL_ASSET)

                    // The following default settings are available to un-comment and edit as needed to
                    // set parameters for custom models.
                    .setModelLabels(LABELS)
                    //.setIsModelTensorFlow2(true)
                    //.setIsModelQuantized(true)
                    //.setModelInputSize(300)
                    //.setModelAspectRatio(16.0 / 9.0)

                    .build();
            // Set confidence threshold for TFOD recognitions, at any time.
            tfod.setMinResultConfidence(0.55f);
        } else if (processor == aprilTag) {
            aprilTag = new AprilTagProcessor.Builder()

                    // The following default settings are available to un-comment and edit as needed.
                    .setDrawAxes(true)
                    .setDrawCubeProjection(true)
                    .setDrawTagOutline(true)
                    .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                    .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                    .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                    // == CAMERA CALIBRATION ==
                    // If you do not manually specify calibration parameters, the SDK will attempt
                    // to load a predefined calibration for your camera.
                    .setLensIntrinsics(fx, fy, cx, cy)
                    // ... these parameters are fx, fy, cx, cy.

                    .build();

            // Adjust Image Decimation to trade-off detection-range for detection-rate.
            // eg: Some typical detection data using a Logitech C920 WebCam
            // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
            // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
            // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
            // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
            // Note: Decimation can be changed on-the-fly to adapt during a match.
            aprilTag.setDecimation(2);
        }
        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(800, 448));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        builder.setAutoStopLiveView(true);

        // Set and enable the processor.
        builder.addProcessor(processor);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the TFOD processor at any time.
        visionPortal.setProcessorEnabled(processor, true);

    }   // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop() + recognition.getBottom()) / 2;
            Seen = true;
            telemetry.addData("", " ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
            if (x <= 300) {
                telemetry.addData("Left Spike Mark", x);
                LEFT = true;
            } else if (x >= 485) {
                telemetry.addData("Right Spike Mark", x);
                RIGHT = true;
            } else if (x > 300 && x < 485) {
                telemetry.addData("Center Spike Mark", x);
                CENTER = true;
            } else
                telemetry.addData("Could not detect the Spike Mark", x);

            Seen = true;
        }   // end for() loop

    }// end method telemetryTfod()
    private void telemetryAprilTag() {


    }   // end method telemetryAprilTag()

}   // end class
