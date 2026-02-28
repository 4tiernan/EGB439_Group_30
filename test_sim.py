from pibot_sim import *

if __name__ == "__main__":
    plt.ion()  # ← turn on interactive mode — opens window without blocking

    bot = PiBotSim(
        pose=np.array([0.5, 0.5, 0.0]),
        dt=0.05,
        realtime=False,  # run as fast as possible for testing)
    )

    #bot.setVelocity(-40, 40, duration=2.0)
    #bot.simulate()

    # Square mode
    bot.move(0.2, 0.0, duration=2)  # 0.2 m/s forward, no rotation
    bot.simulate()

    bot.move(0.0, 0.7894, duration=2)  # 0.2 m/s forward, no rotation
    bot.simulate()

    bot.move(0.2, 0.0, duration=2)  # 0.2 m/s forward, no rotation
    bot.simulate()

    bot.move(0.0, 0.7894, duration=2)  # 0.2 m/s forward, no rotation
    bot.simulate()

    bot.move(0.2, 0.0, duration=2)  # 0.2 m/s forward, no rotation
    bot.simulate()

    bot.move(0.0, 0.7894, duration=2)  # 0.2 m/s forward, no rotation
    bot.simulate()

    bot.move(0.2, 0.0, duration=2)  # 0.2 m/s forward, no rotation
    bot.simulate()

    bot.move(0.0, 0.7894, duration=2)  # 0.2 m/s forward, no rotation
    bot.simulate()





    # #bot.setVelocity(0, 40, duration=2.0)
    # bot.move(0.2, 0.8, duration=2)  # 0.2 m/s forward, 0.8 rad/s rotation
    # bot.simulate()

    # #bot.setVelocity(-40, 0, duration=2.0)
    # bot.move(0.2, -0.8, duration=2)  
    # bot.simulate()


    plt.ioff()   # turn off interactive mode
    plt.show()   # keep window open after loop finishes