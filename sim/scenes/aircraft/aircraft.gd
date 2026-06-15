extends Node3D

var _control_input_speed: float = 0.5
var _model: FESixDofModel = FESixDofModel.new()
var _has_exploded : bool = false

func _ready() -> void:
	_model.set_airspace_velocity(Vector3.FORWARD * 20)
	_model.throttle = 0.5
	_model.elevator_deflector = -0.5
	pass

func _process(delta: float) -> void:
	process_input(delta)
	
	if _has_exploded:
		return
	
	self._model.compute_and_integrate_dynamics(delta);
	var pos : Vector3 = _model.get_position()
	var rot : Vector3 = _model.get_rotation()
	
	_has_exploded = ! pos.is_finite() or ! rot.is_finite()
	if _has_exploded:
		push_error("Model exploded. Yay. :P")
		
	self.position = pos
	self.rotation = rot


func process_input(delta: float) -> void:
	if Input.is_action_pressed("aileron_up"):
		self._model.aileron_deflector += _control_input_speed * delta
	elif Input.is_action_pressed("aileron_down"):
		self._model.aileron_deflector -= _control_input_speed * delta
	
	if Input.is_action_pressed("elevator_up"):
		self._model.elevator_deflector += _control_input_speed * delta
	elif Input.is_action_pressed("elevator_down"):
		self._model.elevator_deflector -= _control_input_speed * delta
	
	if Input.is_action_pressed("ruddler_up"):
		self._model.rudder_deflector += _control_input_speed * delta
	elif Input.is_action_pressed("ruddler_down"):
		self._model.rudder_deflector -= _control_input_speed * delta
	
	if Input.is_action_pressed("throttle_increase"):
		self._model.throttle += _control_input_speed * delta
	elif Input.is_action_pressed("throttle_decrease"):
		self._model.throttle -= _control_input_speed * delta
	
	Sim.get_main_ui().set_control_gauges(
		self._model.aileron_deflector,
		self._model.elevator_deflector,
		self._model.rudder_deflector,
		self._model.throttle
	)
