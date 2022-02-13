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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Husky TeleOpMode", group = "TeleOp")
public class HuskyTeleOpMode extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    HuskyBot huskyBot = new HuskyBot();

    final double END_GAME_TIME = 85.0;  // last 35 seconds
    final double FINAL_TIME = 110.0;    // last 10 seconds
    boolean endGameRumbled = false;
    boolean finalRumbled = false;

    double spinnerPower = 0.0;

    @Override
    public void runOpMode() {
        huskyBot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        boolean intakeOn = false;
        boolean armControlGamepad1 = true;
        boolean curveDrive = false; // used to switch between linear power vs curvilinear
        boolean enableSecondaryDrive = false; // used to grant control to
        int armTarget = huskyBot.arm.getCurrentPosition();

        double y, x, rx;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
//            if ((huskyBot.distanceSensorFrontLeft.getDistance(DistanceUnit.MM) < 20) ||
//                    (huskyBot.distanceSensorFrontRight.getDistance(DistanceUnit.MM) < 20)) {
//                gamepad1.rumbleBlips(1);
//            }

            if ((runtime.seconds() > END_GAME_TIME) && !endGameRumbled)  {
                gamepad1.rumble(1000);
                endGameRumbled = true;
            }

            if ((runtime.seconds() > FINAL_TIME) && !finalRumbled)  {
                gamepad1.rumble(1000);
                finalRumbled = true;
            }

            if (gamepad2.y) {
                enableSecondaryDrive = !enableSecondaryDrive;
            }

            // drive mechanism
            y = -gamepad1.left_stick_y; // Remember, this is reversed!
            x = gamepad1.left_stick_x;
            rx = gamepad1.right_stick_x;

            if (enableSecondaryDrive) {
                y -= 0.3 * (gamepad2.left_stick_y);
                y = Range.clip(y, -1, 1);
                x += 0.3 * (gamepad2.left_stick_x);
                x = Range.clip(x, -1, 1);
                rx += 0.3 * (gamepad2.right_stick_x);
                rx = Range.clip(rx, -1, 1);
            }

            if (gamepad1.dpad_up) {
                curveDrive = true;
            } else if (gamepad1.dpad_down) {
                curveDrive = false;
            }

            if (curveDrive) {
                // smoothen the drive by squaring the values, keeping the sign
//                y = y * Math.abs(y);
//                x = x * Math.abs(x);
//                rx = rx * Math.abs(rx);
                // smoothen the drive by cubing the values
                y = y * y * y;
                x = x * x * x;
                rx = rx * rx * rx;
            }

            if (gamepad2.x) {
                armControlGamepad1 = false;
                huskyBot.arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            if (gamepad2.b) {
                armControlGamepad1 = true;
                huskyBot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            double frontLeftVelocity = (y + x + rx) * HuskyBot.VELOCITY_CONSTANT;
            double rearLeftVelocity = (y - x + rx) * HuskyBot.VELOCITY_CONSTANT;
            double frontRightVelocity = (y - x - rx) * HuskyBot.VELOCITY_CONSTANT;
            double rearRightVelocity = (y + x - rx) * HuskyBot.VELOCITY_CONSTANT;

            // apply the calculated values to the motors.
            huskyBot.frontLeftDrive.setVelocity(frontLeftVelocity);
            huskyBot.rearLeftDrive.setVelocity(rearLeftVelocity);
            huskyBot.frontRightDrive.setVelocity(frontRightVelocity);
            huskyBot.rearRightDrive.setVelocity(rearRightVelocity);

            // arm mechanism, by default controlled by gamepad1 unless overridden by gamepad2
            if (armControlGamepad1) {
                if (gamepad1.a) {
                    armTarget = HuskyBot.ARM_LEVEL_0; // On the ground for starting and intaking
                } else if (gamepad1.x) {
                    armTarget = HuskyBot.ARM_LEVEL_1; // Low level on the goal
                } else if (gamepad1.y) {
                    armTarget = HuskyBot.ARM_LEVEL_2; // Mid level on the goal
                } else if (gamepad1.b) {
                    armTarget = HuskyBot.ARM_LEVEL_3; // High level on the goal
                }
                huskyBot.arm.setTargetPosition(armTarget);
                huskyBot.arm.setVelocity(300); // max velocity
            } else {
                // allow the arm to move freely without encoder target taking input from gamepad2 (if
                // encoders don't work for some reason)
                double armPower = -gamepad2.right_stick_y;
                armPower = Range.clip(armPower, -0.1, 0.35);

                // set power to 0 if any of the limits have been reached
                if ((huskyBot.arm.getCurrentPosition() <= HuskyBot.ARM_LOW_LIMIT && armPower <= 0)
                        || (huskyBot.arm.getCurrentPosition() >= HuskyBot.ARM_HIGH_LIMIT && armPower >= 0)) {
                    armPower = 0;
                }

                // set power to 0 if touch sensor is pressed
                if (huskyBot.armLimit.isPressed() && armPower < 0) {
                    armPower = 0;
                }
                if (armPower == 0) {
                    armPower = 0.1; // so that arm doesn't fall down by gravity
                }

                huskyBot.arm.setPower(armPower);
            }

            if (gamepad1.left_bumper) {
                intakeOn = true;
            } else if (gamepad1.right_bumper) {
                intakeOn = false;
            }

            if (intakeOn) {
                huskyBot.intaker.setPower(HuskyBot.INTAKER_POWER_IN);
            }
            else {
                // set intaker power only if the trigger is more than 0.3 pressed to ignore
                // accidental presses while driving
                double intakerPower = (gamepad1.right_trigger > 0.3) ?
                        HuskyBot.INTAKER_POWER_IN :
                        ((gamepad1.left_trigger > 0.3) ?
                                HuskyBot.INTAKER_POWER_OUT :
                                0);

                // allow the intaker to be controller by gamepad2 as well if needed
                double intakerPower2 = (gamepad2.right_trigger > 0.3) ?
                        HuskyBot.INTAKER_POWER_IN :
                        ((gamepad2.left_trigger > 0.3) ?
                                HuskyBot.INTAKER_POWER_OUT :
                                0);

                // preference is given to gamepad1 values if not 0
                huskyBot.intaker.setPower((intakerPower != 0) ? intakerPower : intakerPower2);
            }

            // spinner mechanism, increase power gradually
            if (gamepad1.dpad_left) {
                spinnerPower += HuskyBot.SPINNER_POWER_INCREMENT;
                if (spinnerPower >= HuskyBot.SPINNER_POWER) spinnerPower = HuskyBot.SPINNER_POWER;
                huskyBot.spinner.setPower(spinnerPower);
            }
            else if (gamepad1.dpad_right) {
                spinnerPower -= HuskyBot.SPINNER_POWER_INCREMENT;
                if (spinnerPower <= -HuskyBot.SPINNER_POWER) spinnerPower = -HuskyBot.SPINNER_POWER;
                huskyBot.spinner.setPower(spinnerPower);
            }
            else {
                spinnerPower = 0.0;
                huskyBot.spinner.setPower(0);
            }
//            huskyBot.spinner.setPower(gamepad1.dpad_left ?
//                    HuskyBot.SPINNER_POWER : gamepad1.dpad_right ? -HuskyBot.SPINNER_POWER : 0);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Actual Vel", "fl (%.2f), rl (%.2f)", huskyBot.frontLeftDrive.getVelocity(), huskyBot.rearLeftDrive.getVelocity());
            telemetry.addData("Actual Vel", "fr (%.2f), rr (%.2f)", huskyBot.frontRightDrive.getVelocity(), huskyBot.rearRightDrive.getVelocity());
            telemetry.addData("Target Vel", "fl (%.2f), rl (%.2f)", frontLeftVelocity, rearLeftVelocity);
            telemetry.addData("Target Vel", "fr (%.2f), rr (%.2f)", frontRightVelocity, rearRightVelocity);
            telemetry.addData("Power", "front left (%.2f), rear left (%.2f)", huskyBot.frontLeftDrive.getPower(), huskyBot.rearLeftDrive.getPower());
            telemetry.addData("Power", "front right (%.2f), rear right (%.2f)",  huskyBot.frontLeftDrive.getPower(), huskyBot.rearLeftDrive.getPower());
            telemetry.addData("Arm", "Power (%.2f), Encoder Value: (%d)", huskyBot.arm.getPower(), huskyBot.arm.getCurrentPosition());
            telemetry.addData("Arm", "Target (%d), Target Position: (%d)", armTarget, huskyBot.arm.getTargetPosition());
            telemetry.addData("Arm", "Velocity (%.2f), Mode (%s)", huskyBot.arm.getVelocity(), huskyBot.arm.getMode());
            telemetry.addData("ArmLimit", huskyBot.armLimit.isPressed());
            telemetry.addData("Arm Control Gamepad 1 (%b)", armControlGamepad1);

            telemetry.addData("Stick", "y (%.2f), x (%.2f), rx (%.2f)", y, x, rx);
            telemetry.addData("Velocity", "front left (%.2f), rear left (%.2f)", frontLeftVelocity, rearLeftVelocity);
            telemetry.addData("Velocity", "front right (%.2f), rear right (%.2f)", frontRightVelocity, rearRightVelocity);

            telemetry.addData("Distance Front Left", huskyBot.distanceSensorFrontLeft.getDistance(DistanceUnit.MM));
            telemetry.addData("Distance Front Right", huskyBot.distanceSensorFrontRight.getDistance(DistanceUnit.MM));
            telemetry.addData("Distance Right", huskyBot.distanceSensorRight.getDistance(DistanceUnit.MM));
            telemetry.addData("Distance Back", huskyBot.distanceSensorBack.getDistance(DistanceUnit.MM));
            telemetry.addData("Distance Left", huskyBot.distanceSensorLeft.getDistance(DistanceUnit.MM));

            telemetry.update();
        }
    }
}
