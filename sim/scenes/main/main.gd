extends Node
class_name Main

var _is_paused: bool = false

func _ready() -> void:
	DisplayServer.window_set_mode(DisplayServer.WINDOW_MODE_EXCLUSIVE_FULLSCREEN)
	
	Sim.initialize(self)
	Sim.load_flight_scene()

func _process(delta: float) -> void:
	if Input.is_action_just_pressed("pause_sim"):
		self._is_paused = ! self._is_paused
		if(self._is_paused):
			Sim.get_active_scene().process_mode = Node.PROCESS_MODE_DISABLED
			Sim.get_main_ui().set_pause_icon_visible(self._is_paused)
		else:
			Sim.get_active_scene().process_mode = Node.PROCESS_MODE_ALWAYS
			Sim.get_main_ui().set_pause_icon_visible(self._is_paused)
