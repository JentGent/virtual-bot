
WHEEL_RADIUS = 2.75;

angle = 30.48 * 360/(2*pi*WHEEL_RADIUS);
brick.MoveMotorAngleRel('AD', 50, angle, 1);
brick.WaitForMotor('A');
brick.MoveMotorAngleRel('AD', 50, -angle, 1);
