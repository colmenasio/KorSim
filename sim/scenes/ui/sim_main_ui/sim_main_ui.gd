extends CanvasLayer
class_name SimMainUI

#ts is lazy asf but meh
func set_control_gauges(da: float, de: float, dr: float, tt: float) -> void:
	%ControlBarDA.value = da
	%ControlBarDE.value = de
	%ControlBarDR.value = dr
	%ControlBarTT.value = tt

func set_gyro_compass_rotation(rot: Vector3):
	%GyroCompass.set_compass_rotation(rot)

func set_pause_icon_visible(value: bool):
	%PauseIcon.visible = value

func set_info_display_airspeed(value: float):
	$MarginContainer/VBoxContainer/StatsViualization/MarginContainer/Stats/SpeedLabel.text = "Airspeed Module %d" % value 

func set_attack_angle_gauge(value: float):
	var t = (clamp(value, -1, 1) * 0.5) + 0.5
	var gauge: Control = %AttackAngleGauge
	var gauge_parent : Control = gauge.get_parent()
	gauge.position.y = gauge_parent.size.y * t
