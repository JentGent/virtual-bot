
WHEEL_RADIUS = 2.75; % use wheelRadiusTest.m to find radius first
WHEEL_DIST = 7.9; % if it turns past 360, lower this; if <360, raise it

MOTOR = 'A';

angle = WHEEL_DIST * 360 / WHEEL_RADIUS;
brick.MoveMotorAngleRel(MOTOR, 50, angle, 1);
brick.WaitForMotor(MOTOR);
brick.MoveMotorAngleRel(MOTOR, 50, -angle, 1);
