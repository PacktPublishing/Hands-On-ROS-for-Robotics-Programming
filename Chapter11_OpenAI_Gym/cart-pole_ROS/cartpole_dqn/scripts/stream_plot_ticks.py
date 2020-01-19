#!/usr/bin/env python
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import random

import rospy
from std_msgs.msg import Int16MultiArray, Int32

# Initialize the ROS node
rospy.init_node('ticks_graph')


def callback_reward(msg):
    rospy.logwarn(' EPISODE ' + str(msg.data[0]) + ' Ticks= ' + str(msg.data[1]))
    xs.append(msg.data[0])
    ys.append(msg.data[1])
sub = rospy.Subscriber('ticks', Int16MultiArray, callback_reward)

# Create figure for plotting
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
xs = []
ys = []
#ys.append(30)
# This function is called periodically from FuncAnimation
def animate(i, xs, ys):

    # Generate a random temperature
    #temp_c = round(random.randint(1,101), 2)

    # Add x and y to lists
    #xs.append(i)
    #ys.append(temp_c)

    # Limit x and y lists to 20 items
    #xs = xs[-20:]
    #ys = ys[-20:]

    # Draw x and y lists
    ax.clear()
    ax.plot(xs, ys)

    # Format plot
    plt.xticks(rotation=45, ha='right')
    plt.subplots_adjust(bottom=0.30)
    plt.title('Cart-Pole training with DQN-learn: TICKS')
    plt.ylabel('Total ticks')
    plt.xlabel('Episode number')
# Set up plot to call animate() function periodically
ani = animation.FuncAnimation(fig, animate, fargs=(xs, ys), interval=1000)
plt.show()

rospy.spin()