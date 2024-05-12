# def live_plot(log_reader: LogReader) -> None:
#     fig, ax = plt.subplots(3, 1)
#     x_line, = ax[0].plot([], color='r')
#     y_line, = ax[1].plot([], color='r')
#     theta_line, = ax[2].plot([], color='r')

#     def update_lines(i):
#         t, x, y, theta, v, w = log_reader.get_latest_vals()
#         x_line.set_data(log_reader.time_vals[:i], log_reader.x_vals[:i])
#         y_line.set_data(log_reader.time_vals[:i], log_reader.y_vals[:i])
#         theta_line.set_data(log_reader.time_vals[:i], log_reader.theta_vals[:i])
#         return x_line, y_line, theta_line

#     ani = animation.FuncAnimation(fig, update_lines,
#                                    frames=len(log_reader.time_vals),
#                                    interval=50,
#                                    blit=True)
#     plt.show()