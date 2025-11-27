
simulated = true;

if simulated
    hw = HardwareSim(6, 3);
else
    hw = Hardware(brick);
end
maze = Maze(11, 11);
odo = Odometry(hw, maze);
Gsize = maze.CELL_SIZE * (max(maze.MAZE_WIDTH, maze.MAZE_HEIGHT));
odo.setPose(Gsize / 2, Gsize / 2, 0);
viz = Visualization(hw, odo, maze);
bot = Bot(hw, maze, odo, viz);

if simulated
    hw.REAL_MAZE.draw(hw.viz.ax);
end

bot.run();
