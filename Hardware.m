classdef Hardware < handle
    properties (Constant)
        MOTOR = struct( ...
            'LEFT', 'A', ...
            'RIGHT', 'D', ...
            'CLAW', 'C' ...
        )
        SENSOR = struct( ...
            'TOUCH', 1, ...
            'COLOR', 2, ...
            'ULTRASONIC', 3, ...
            'GYRO', 4 ...
        )
        WHEEL_RADIUS = 2.75
        WHEEL_DIST = 7.9
        xUS = 5.8
        yUS = -4.2
        yTouch = 9.5
    end
    properties
        brick
    end

    methods
        function obj = Hardware(brick)
            brick.SetColorMode(obj.SENSOR.COLOR, 4); % rgb
            obj.brick = brick;
            obj.beep();
        end

        function beep(obj)
            obj.brick.beep();
        end
        function move(obj, left, right)
            b = obj.brick;
            b.MoveMotor(obj.MOTOR.LEFT, left);
            b.MoveMotor(obj.MOTOR.RIGHT, right);
        end
        function moveClaw(obj, power)
            obj.brick.MoveMotor(obj.MOTOR.CLAW, -power);
        end
        function stop(obj)
            obj.brick.StopAllMotors();
            obj.brick.WaitForMotor(obj.MOTOR.LEFT);
        end
        function angles = getAngles(obj)
            brick = obj.brick;
            MOTOR = obj.MOTOR;
            angles = [brick.GetMotorAngle(MOTOR.LEFT), brick.GetMotorAngle(MOTOR.RIGHT)];
        end
        function steer(obj, speed, radius)
            outerRadius = abs(radius) + obj.WHEEL_DIST / 2;
            innerRadius = abs(radius) - obj.WHEEL_DIST / 2;
            ratio = innerRadius / outerRadius;
            if radius >= 0
                leftPower = speed;
                rightPower = speed * ratio;
            else
                leftPower = speed * ratio;
                rightPower = speed;
            end
            obj.move(leftPower, rightPower);
        end
        function resetMotors(obj)
            b = obj.brick;
            b.ResetMotorAngle(obj.MOTOR.LEFT);
            b.ResetMotorAngle(obj.MOTOR.RIGHT);
        end
        function pressed = touchPressed(obj)
            pressed = obj.brick.TouchPressed(obj.SENSOR.TOUCH);
        end
        function color = getColor(obj)
            color = obj.brick.ColorRGB(obj.SENSOR.COLOR);
        end
        function dist = rightDist(obj)
            dist = obj.brick.UltrasonicDist(obj.SENSOR.ULTRASONIC);
        end
    end
end