package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp

public class Payload_TeleOp extends LinearOpMode {
    private DcMotor fr, fl, br, bl, dcAc1, dcAc2, dcIn1, dcIn2;
    private Servo serAc1, serAc2;
    private CRServo serIn1, serIn2, serIn3;

    public void runOpMode() {
        fl = hardwareMap.get(DcMotor.class, "fl");
        br = hardwareMap.get(DcMotor.class, "br");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        dcAc1 = hardwareMap.get(DcMotor.class, "dcAc1");
        dcAc2 = hardwareMap.get(DcMotor.class, "dcAc2");
        dcIn1 = hardwareMap.get(DcMotor.class, "dcIn1");
        dcIn2 = hardwareMap.get(DcMotor.class, "dcIn2");
        serAc1 = hardwareMap.get(Servo.class, "serAc1");
        serAc2 = hardwareMap.get(Servo.class, "serAc2");
        serIn1 = hardwareMap.get(CRServo.class, "serIn1");
        serIn2 = hardwareMap.get(CRServo.class, "serIn2");
        serIn3 = hardwareMap.get(CRServo.class, "serIn3");
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        serAc2.setDirection(Servo.Direction.REVERSE);
        serAc1.setPosition(0);
        serAc2.setPosition(0);
        telemetry.addLine("Motors Assigned and Attached");
        telemetry.addLine(String.valueOf(serAc1.getPosition()));
        telemetry.addLine(String.valueOf(serAc2.getPosition()));
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            double lx = gamepad1.left_stick_x;
            double ly = gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;
            double rt = gamepad2.right_trigger;
            double lt = gamepad2.left_trigger;
            double ly2 = gamepad2.left_stick_y;
            double ry2 = gamepad2.right_stick_y;
            if (gamepad1.right_bumper == false) {
                fl.setPower(ly + lx - rx);
                fr.setPower(ly - lx + rx);
                bl.setPower(ly - lx - rx);
                br.setPower(ly + lx + rx);
            } else {
                if (gamepad1.right_bumper == true) {
                    fl.setPower((ly + lx - rx) / 2);
                    fr.setPower((ly - lx + rx) / 2);
                    bl.setPower((ly - lx - rx) / 2);
                    br.setPower((ly + lx + rx) / 2);
                }
            }
            dcAc1.setPower(-ly2);
            dcAc2.setPower(-ry2);
            serIn1.setPower(rt);
            serIn2.setPower(-rt);
            serIn1.setPower(-lt);
            serIn2.setPower(lt);

            if (gamepad1.right_bumper == true) {

            }

            if (gamepad1.dpad_up || gamepad2.dpad_up == true) {
                dcAc1.setPower(1);
                dcAc2.setPower(1);
            }
            if (gamepad2.dpad_up == false || gamepad1.dpad_up == false ) {
                dcAc1.setPower(0);
                dcAc2.setPower(0);
            }
            if (gamepad2.dpad_down == true || gamepad1.dpad_down == true) {
                while (gamepad2.dpad_down == true) {
                    dcAc1.setPower(-1);
                    dcAc2.setPower(-1);
                }
                if (gamepad2.dpad_up == false || gamepad1.dpad_down == false) {
                    dcAc1.setPower(0);
                    dcAc2.setPower(0);
                }
            }
            if (gamepad2.y == true) {
                serAc1.setPosition(0.35);
                serAc2.setPosition(0.35);
            }
            if (gamepad2.a == true) {
                serAc1.setPosition(0);
                serAc2.setPosition(0);
            }
            while (gamepad2.right_bumper == true) {
                dcIn2.setPower(0.5);
                if (gamepad2.right_bumper == false) {
                    dcIn2.setPower(0.5);
                }
            }
            while (gamepad2.left_bumper == true) {
                dcIn2.setPower(0.5);
                if(gamepad2.left_bumper == false) {
                    dcIn2.setPower(0);
                }
            }
            while (gamepad2.x == true) {
                dcIn2.setPower(-1);
                dcIn1.setPower(-1);
                if (gamepad2.x == false) {
                    dcIn2.setPower(0);
                    dcIn2.setPower(0);
                }
            }
            while (gamepad2.b== true) {
                dcIn2.setPower(1);
                dcIn1.setPower(1);
                if (gamepad2.b == false) {
                    dcIn2.setPower(0);
                    dcIn1.setPower(0);
                }
            }
            while (gamepad2.dpad_left == true) {
                serIn3.setPower(1);
                if (gamepad2.dpad_left == false) {
                    serIn3.setPower(0);
                }
            }
            while (gamepad2.dpad_left == true) {
                serIn3.setPower(-1);
                if (gamepad2.dpad_left == false) {
                    serIn3.setPower(0);
                }
            }
        }
    }
}