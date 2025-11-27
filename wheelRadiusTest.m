
WHEEL_RADIUS = 2.75; % if it moves over 1ft, lower this; if <1ft, raise it

MOTOR1 = 'A';
MOTOR2 = 'D';

angle = 30.48 * 360/(2*pi*WHEEL_RADIUS);
brick.MoveMotorAngleRel([MOTOR1, MOTOR2], 50, angle, 1);
brick.WaitForMotor(MOTOR1);
brick.WaitForMotor(MOTOR2);
brick.MoveMotorAngleRel([MOTOR1, MOTOR2], 50, -angle, 1);
