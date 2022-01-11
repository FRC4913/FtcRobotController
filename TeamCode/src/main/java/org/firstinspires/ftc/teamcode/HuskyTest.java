/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "Husky Test", group = "TeleOp")
public class HuskyTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    HuskyBot huskyBot = new HuskyBot();
    double rearLeftCurrentVel, rearRightCurrentVel, frontLeftCurrentVel, frontRightCurrentVel;
    double rearLeftMaxVel, rearRightMaxVel, frontLeftMaxVel, frontRightMaxVel = 0.0;

    @Override
    public void runOpMode() {
        huskyBot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        huskyBot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        huskyBot.frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        huskyBot.rearLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        huskyBot.rearRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        runtime.reset();

        int motor = 0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // drive mechanism
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!

            double power = Range.clip(y, -1.0, 1.0);

            if (gamepad1.a) {
                motor = 0;
            } else if (gamepad1.x) {
                motor = 1;
            } else if (gamepad1.y) {
                motor = 2;
            } else if (gamepad1.b) {
                motor = 3;
            }

            switch (motor) {
                case 0:
                    huskyBot.frontLeftDrive.setPower(power);
                    break;
                case 1:
                    huskyBot.rearLeftDrive.setPower(power);
                    break;
                case 2:
                    huskyBot.frontRightDrive.setPower(power);
                    break;
                case 3:
                    huskyBot.rearRightDrive.setPower(power);
                    break;
                default:
                    huskyBot.frontLeftDrive.setPower(0);
                    huskyBot.rearLeftDrive.setPower(0);
                    huskyBot.frontRightDrive.setPower(0);
                    huskyBot.rearRightDrive.setPower(0);
                    break;
            }

            rearLeftCurrentVel = huskyBot.rearLeftDrive.getVelocity();
            if (rearLeftCurrentVel > rearLeftMaxVel) {
                rearLeftMaxVel = rearLeftCurrentVel;
            }

            rearRightCurrentVel = huskyBot.rearRightDrive.getVelocity();
            if (rearRightCurrentVel > rearRightMaxVel) {
                rearRightMaxVel = rearRightCurrentVel;
            }

            frontLeftCurrentVel = huskyBot.frontLeftDrive.getVelocity();
            if (frontLeftCurrentVel > frontLeftMaxVel) {
                frontLeftMaxVel = frontLeftCurrentVel;
            }

            frontRightCurrentVel = huskyBot.frontRightDrive.getVelocity();
            if (frontRightCurrentVel > frontRightMaxVel) {
                frontRightMaxVel = frontRightCurrentVel;
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Velocity", "front left (%.2f), rear left (%.2f)", huskyBot.frontLeftDrive.getVelocity(), huskyBot.rearLeftDrive.getVelocity());
            telemetry.addData("Velocity", "front right (%.2f), rear right (%.2f)", huskyBot.frontRightDrive.getVelocity(), huskyBot.rearRightDrive.getVelocity());
//            telemetry.addData("Motors", "front left max velocity (%.2f)", huskyBot.frontLeftDrive.getVelocity(AngleUnit.DEGREES));
            telemetry.addData("Motors", "front left max velocity (%.2f)", frontLeftMaxVel);
            telemetry.addData("Motors", "rear left max velocity (%.2f)", rearLeftMaxVel);
            telemetry.addData("Motors", "front right max velocity (%.2f)", frontRightMaxVel);
            telemetry.addData("Motors", "rear right max velocity (%.2f)", rearRightMaxVel);
            telemetry.addData("Motors", "current (%.2f)", huskyBot.frontRightDrive.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Motors", "current position (%d)", huskyBot.frontRightDrive.getCurrentPosition());
            telemetry.addData("Motors", "front right power (%.2f)", huskyBot.frontRightDrive.getPower());
            telemetry.addData("Motors", "front left power (%.2f)", huskyBot.frontLeftDrive.getPower());
            telemetry.addData("Motors", "rear right power (%.2f)", huskyBot.rearRightDrive.getPower());
            telemetry.addData("Motors", "rear left power (%.2f)", huskyBot.rearLeftDrive.getPower());

//            telemetry.addData("current velocity", currentVelocity);
//            telemetry.addData("maximum velocity", maxVelocity);
            telemetry.update();
        }
    }
}
