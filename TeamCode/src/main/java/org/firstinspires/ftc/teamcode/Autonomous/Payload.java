package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.sql.Time;
import java.util.List;
import java.util.concurrent.TimeUnit;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class Payload extends LinearOpMode {
    final double DESIRED_DISTANCE = 12.0;
    // ... (other constants)

    public DcMotor fl, fr, bl, br;

    // ... (other member variables)

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();

        waitForStart();

        while (opModeIsActive()) {
            boolean targetFound = false;    // Set to true when an AprilTag target is detected
            double drive = 0;        // Desired forward power/speed (-1 to +1)
            double strafe = 0;        // Desired strafe power/speed (-1 to +1)
            double turn = 0;        // Desired turning power/speed (-1 to +1)
            executeAutonomous();
        }
    }

    private void initHardware() {
        // Initialize motors and other hardware
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");

        // Set motor directions
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);
    }

    private void executeAutonomous() throws InterruptedException {
        fr.setPower(1);
        fl.setPower(1);
        bl.setPower(1);
        br.setPower(1);
        TimeUnit.MILLISECONDS.sleep(200);
        fr.setPower(0);
        fl.setPower(0);
        bl.setPower(0);
        br.setPower(0);
        telemetry.addLine("Robot Moved Forward");
        telemetry.update();
        fr.setPower(1);
        br.setPower(1);
        fl.setPower(-1);
        bl.setPower(-1);
        TimeUnit.MILLISECONDS.sleep(500);
        fr.setPower(0);
        br.setPower(0);
        fl.setPower(0);
        bl.setPower(0);
        //moveRobot();
    }

    private void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers
        double leftFrontPower = 0, rightFrontPower = 0, leftBackPower = 0, rightBackPower = 0;
        // ...

        // Normalize wheel powers
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels
        fl.setPower(leftFrontPower);
        fr.setPower(rightFrontPower);
        bl.setPower(leftBackPower);
        br.setPower(rightBackPower);
    }

    private void initAprilTag() {

    }
}
