
WHEEL_RADIUS = 2.75;
WHEEL_DIST = 7.9;

angle = WHEEL_DIST * 360 / WHEEL_RADIUS;
brick.MoveMotorAngleRel('A', 50, angle, 1);
brick.WaitForMotor('A');
brick.MoveMotorAngleRel('A', 50, -angle, 1);
