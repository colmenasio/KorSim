extends Node
class_name Main

func _ready() -> void:
	DisplayServer.window_set_mode(DisplayServer.WINDOW_MODE_EXCLUSIVE_FULLSCREEN)
	
	Sim.initialize(self)
	Sim.load_flight_scene()
