extends CanvasLayer
class_name SimMainUI

#ts is lazy asf but meh
func set_control_gauges(da: float, de: float, dr: float, tt: float) -> void:
	%ControlBarDA.value = da
	%ControlBarDE.value = de
	%ControlBarDR.value = dr
	%ControlBarTT.value = tt
