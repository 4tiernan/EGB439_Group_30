from pibot_sim import *

if __name__ == "__main__":
    plt.ion()  # ← turn on interactive mode — opens window without blocking

    bot = PiBotSim(
        pose=np.array([0.5, 0.5, 0.0]),
        dt=0.05,
        realtime=True,  # run as fast as possible for testing)
    )

    # Helper function to print pose in a nice format
    def print_pose(bot):
        print(f"X: {np.round(bot.pose[0], 2)}  "
              f"Y: {np.round(bot.pose[1], 2)}  "
              f"θ: {np.rad2deg(np.round(bot.pose[2], 2)):.1f}°")
        

    # Square mode: for loop for 4 sides + 4 turns
    for side in range(4):
        print(f"--- Straight {side + 1} ---")
        bot.move(0.5, 0.0, duration=2.0)
        bot.simulate()
        print_pose(bot)

        print(f"--- Corner {side + 1} ---")
        bot.move(0.0, 1.5708, duration=1.0)  # pi/4 rad/s for 1 second = 90 degree turn
        bot.simulate()
        print_pose(bot)

    print("Square complete!")
    plt.ioff()
    plt.show()





    #bot.setVelocity(-40, 40, duration=2.0)
    #bot.simulate()

    # # Square mode
    # bot.move(0.2, 0.0, duration=2)  # 0.2 m/s forward, no rotation
    # bot.simulate()
    # print("Position [X]:", np.round(bot.pose[0], 2), "Position [Y]:", np.round(bot.pose[1], 2), "Orientation [theta]:", np.rad2deg(np.round(bot.pose[2], 2)))

    # bot.move(0.0, 0.7894, duration=2) # 0.7894 rad/s corresponds to 90 degrees in 2 seconds 
    # bot.simulate()
    # print("Position [X]:", np.round(bot.pose[0], 2), "Position [Y]:", np.round(bot.pose[1], 2), "Orientation [theta]:", np.rad2deg(np.round(bot.pose[2], 2)))


    # bot.move(0.2, 0.0, duration=2)  
    # bot.simulate()
    # print("Position [X]:", np.round(bot.pose[0], 2), "Position [Y]:", np.round(bot.pose[1], 2), "Orientation [theta]:", np.rad2deg(np.round(bot.pose[2], 2)))

    # bot.move(0.0, 0.7894, duration=2)  
    # bot.simulate()
    # print("Position [X]:", np.round(bot.pose[0], 2), "Position [Y]:", np.round(bot.pose[1], 2), "Orientation [theta]:", np.rad2deg(np.round(bot.pose[2], 2)))

    # bot.move(0.2, 0.0, duration=2)  
    # bot.simulate()
    # print("Position [X]:", np.round(bot.pose[0], 2), "Position [Y]:", np.round(bot.pose[1], 2), "Orientation [theta]:", np.rad2deg(np.round(bot.pose[2], 2)))

    # bot.move(0.0, 0.7894, duration=2)  
    # bot.simulate()
    # print("Position [X]:", np.round(bot.pose[0], 2), "Position [Y]:", np.round(bot.pose[1], 2), "Orientation [theta]:", np.rad2deg(np.round(bot.pose[2], 2)))

    # bot.move(0.2, 0.0, duration=2)  
    # bot.simulate()
    # print("Position [X]:", np.round(bot.pose[0], 2), "Position [Y]:", np.round(bot.pose[1], 2), "Orientation [theta]:", np.rad2deg(np.round(bot.pose[2], 2)))

    # bot.move(0.0, 0.7894, duration=2)  
    # bot.simulate()
    # print("Position [X]:", np.round(bot.pose[0], 2), "Position [Y]:", np.round(bot.pose[1], 2), "Orientation [theta]:", np.rad2deg(np.round(bot.pose[2], 2)))
    # print("Square complete!")






    # #bot.setVelocity(0, 40, duration=2.0)
    # bot.move(0.2, 0.8, duration=2)  # 0.2 m/s forward, 0.8 rad/s rotation
    # bot.simulate()

    # #bot.setVelocity(-40, 0, duration=2.0)
    # bot.move(0.2, -0.8, duration=2)  
    # bot.simulate()


    plt.ioff()   # turn off interactive mode
    plt.show()   # keep window open after loop finishes