## @ingroup robotcontrol_msgs
# @defgroup ServoDiag ServoDiag
#
# &nbsp;
#
# Message type used to publish current diagnostics about an individual servo, such as
# communication statistics.
#
# @param "string name" Name of the servo
# @param "uint32 timeouts" Number of servo communication timeouts
# @param "uint32 checksum_errors" Number of received packet checksum errors

string name
uint32 timeouts
uint32 checksum_errors
