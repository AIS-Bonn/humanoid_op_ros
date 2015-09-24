timewarp   {#timewarp}
========

`timewarp` subscribes to specified topics and maintains received messages in
a circular buffer. It also provides a copy of each topic under the `/vis/`
prefix.

Using a service call (or the plotter GUI) the message stream on the `/vis/`
topics can be paused and rewound to an earlier point in time (if it is still
contained in the circular buffer).

An example launch file resides in `timewarp/launch/warped_rqt.launch`.
