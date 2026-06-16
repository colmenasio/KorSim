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
