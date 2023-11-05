package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.concurrent.TimeUnit;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp

public class Payload_TeleOp extends LinearOpMode {
    private DcMotor fr, fl, br, bl, dcAc1, dcAc2, dcIn1, dcIn2;
    private Servo serAc1, serAc2;

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
            fl.setPower(ly + lx - rx);
            fr.setPower(ly - lx + rx);
            bl.setPower(ly - lx - rx);
            br.setPower(ly + lx + rx);
            dcIn2.setPower(-rt);
            dcIn2.setPower(lt);
            dcAc1.setPower(-ry2);
            dcAc2.setPower(-ly2);

            if (gamepad1.dpad_up || gamepad2.dpad_up == true) {
                dcAc1.setPower(1);
                dcAc2.setPower(1);
            }
            if (gamepad2.dpad_up == false) {
                dcAc1.setPower(0);
                dcAc2.setPower(0);
            }
            if (gamepad2.dpad_down == true) {
                while (gamepad2.dpad_down == true) {
                    dcAc1.setPower(-1);
                    dcAc2.setPower(-1);
                }
                if (gamepad2.dpad_up == false) {
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
            if (gamepad2.right_bumper == true) {
                try {
                    dcIn1.setPower(1);
                    TimeUnit.MILLISECONDS.sleep(80);
                    dcIn1.setPower(0);
                    TimeUnit.SECONDS.sleep(1);
                } catch (Exception e) {
                    //TODO: handle exception
                }
                if (gamepad2.left_bumper == true) {
                    try {
                        dcIn1.setPower(-1);
                        TimeUnit.MILLISECONDS.sleep(80);
                        dcIn1.setPower(0);
                        TimeUnit.SECONDS.sleep(1);
                    } catch (Exception e) {
                        //TODO: handle exception
                    }
                }
            }
        }
    }
}