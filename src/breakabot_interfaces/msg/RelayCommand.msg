# RelayCommand.msg
# Command the relay board to select a channel for a motor controller.
#
# motor_controller: target motor controller (1, 2, or 3)
# channel:          target channel (1 = normally open, 2 = normally closed)

uint8 motor_controller
uint8 channel