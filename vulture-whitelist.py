"""Names vulture reports as unused although they are used.

Klipper calls the first group by convention; other projects call the second group through the
documented host interface (docs/protocol.md); the third group are fields read by position. The
file is never imported: each line only uses the name once, so vulture counts it.
"""

# Klipper
cmd_ACEK_ASSIST
cmd_ACEK_ASSIST_help
cmd_ACEK_CALIBRATE_ENCODER
cmd_ACEK_CALIBRATE_ENCODER_help
cmd_ACEK_CLEAR
cmd_ACEK_CLEAR_help
cmd_ACEK_DRY
cmd_ACEK_DRY_CLEAR
cmd_ACEK_DRY_CLEAR_help
cmd_ACEK_DRY_help
cmd_ACEK_DRY_STOP
cmd_ACEK_DRY_STOP_help
cmd_ACEK_DRYER_LOG
cmd_ACEK_DRYER_LOG_help
cmd_ACEK_FAN
cmd_ACEK_FAN_help
cmd_ACEK_FEED
cmd_ACEK_FEED_FORWARD
cmd_ACEK_FEED_FORWARD_help
cmd_ACEK_FEED_help
cmd_ACEK_FF_SET
cmd_ACEK_FF_SET_help
cmd_ACEK_FLAP
cmd_ACEK_FLAP_help
cmd_ACEK_FOLLOW_SET
cmd_ACEK_FOLLOW_SET_help
cmd_ACEK_GRIP
cmd_ACEK_GRIP_help
cmd_ACEK_LANE_ACTIVE
cmd_ACEK_LANE_ACTIVE_help
cmd_ACEK_LOAD
cmd_ACEK_LOAD_help
cmd_ACEK_LOAD_SET
cmd_ACEK_LOAD_SET_help
cmd_ACEK_RFID_FORGET
cmd_ACEK_RFID_FORGET_help
cmd_ACEK_RFID_READ
cmd_ACEK_RFID_READ_help
cmd_ACEK_ROLLBACK
cmd_ACEK_ROLLBACK_help
cmd_ACEK_SNAG_SET
cmd_ACEK_SNAG_SET_help
cmd_ACEK_SPEED
cmd_ACEK_SPEED_help
cmd_ACEK_STOP
cmd_ACEK_STOP_help
cmd_ACEK_UNLOAD
cmd_ACEK_UNLOAD_help
get_report_time_delta
load_config
load_config_prefix
setup_callback
setup_minmax

# The host interface (docs/protocol.md, "What a caller can rely on")
default_move
follow_default
follow_paused
gone_after_s
hand_over
hold_follow
move_wait_s
set_lane_follow
set_lane_park
set_lane_wait
set_material_source
shared_lanes
start_bound  # the print mode's (ace2k_dryer_print.py)
start_move
start_read

# Fields read by position
cold_window_s  # BenchTiming (ace2k_dryer_mode.py): sent in the order of TIMING_FMT
link_s  # BenchTiming, likewise
