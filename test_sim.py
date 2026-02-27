from pibot_sim import *

if __name__ == "__main__":
    plt.ion()  # ← turn on interactive mode — opens window without blocking

    bot = PiBotSim(
        pose=np.array([0.5, 0.5, 0.0]),
        dt=0.05,
    )

    t = 0.0
    total = 10.0

    while t < total:
        # first 5 seconds, move forward at X speed, then turn at Y speed for the next 5 seconds
        if t < 5.0:
            left_motor_cmd = 40
            right_motor_cmd = 40
        else:
            left_motor_cmd = 10
            right_motor_cmd = 30

        bot.setVelocity(left_motor_cmd, right_motor_cmd)
        #bot.move(forward_vel, angular_vel)
        bot.step()
        bot.update_plot()
        time.sleep(bot.dt)
        t += bot.dt

    plt.ioff()   # turn off interactive mode
    plt.show()   # keep window open after loop finishes