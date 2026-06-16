extends Node
class_name Main

var _is_paused: bool = false
var _camera_pre_pause: Camera3D = null

func _ready() -> void:
	DisplayServer.window_set_mode(DisplayServer.WINDOW_MODE_EXCLUSIVE_FULLSCREEN)
	
	Sim.initialize(self)
	Sim.load_flight_scene()

func _process(delta: float) -> void:
	pass
	if Input.is_action_just_pressed("pause_sim"):
		self._is_paused = ! self._is_paused
		if(self._is_paused):
			_camera_pre_pause = self.get_viewport().get_camera_3d()
			$DebugCamera3D.movement_active = true
			$DebugCamera3D.global_position = _camera_pre_pause.global_position
			$DebugCamera3D.global_rotation = _camera_pre_pause.global_rotation
			$DebugCamera3D.current = true
			Sim.get_active_scene().process_mode = Node.PROCESS_MODE_DISABLED
			Sim.get_main_ui().set_pause_icon_visible(self._is_paused)
		else:
			$DebugCamera3D.movement_active = false
			_camera_pre_pause.current = true
			Sim.get_active_scene().process_mode = Node.PROCESS_MODE_ALWAYS
			Sim.get_main_ui().set_pause_icon_visible(self._is_paused)
