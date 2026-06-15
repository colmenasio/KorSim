extends SixDofAircraft

var _control_input_speed: float = 0.5

func _ready() -> void:
	pass
	#airspace_velocity = Vector3.FORWARD * 25
	#throttle = 0.5
	#elevator_deflector = 0.8

func _process(delta: float) -> void:
	self.update_transform()
	
	if Input.is_action_pressed("aileron_up"):
		self.aileron_deflector += _control_input_speed * delta
	elif Input.is_action_pressed("aileron_down"):
		self.aileron_deflector -= _control_input_speed * delta
	
	if Input.is_action_pressed("elevator_up"):
		self.elevator_deflector += _control_input_speed * delta
	elif Input.is_action_pressed("elevator_down"):
		self.elevator_deflector -= _control_input_speed * delta
	
	if Input.is_action_pressed("ruddler_up"):
		self.rudder_deflector += _control_input_speed * delta
	elif Input.is_action_pressed("ruddler_down"):
		self.rudder_deflector -= _control_input_speed * delta
	
	if Input.is_action_pressed("throttle_increase"):
		self.throttle += _control_input_speed * delta
	elif Input.is_action_pressed("throttle_decrease"):
		self.throttle -= _control_input_speed * delta
	
	Sim.get_main_ui().set_control_gauges(
		self.aileron_deflector,
		self.elevator_deflector,
		self.rudder_deflector,
		self.throttle
	)

func _physics_process(delta: float) -> void:
	self.update_dynamics(delta)
