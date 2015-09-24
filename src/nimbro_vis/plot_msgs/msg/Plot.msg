## @ingroup plot_msgs
# @defgroup Plot Plot
#
# Message type used to pass plot data to the rqt plotter plugin.
# Usage: Simply publish on the global /plot topic with the data you want to
# plot.
#
# @param "Header header" Standard ROS header (set the timestamp here!)
# @param "PlotPoint[] points" Points to plot for this timestamp (see @ref PlotPoint)
# @param "string[] events" Event markers to display


Header header
PlotPoint[] points
string[] events
