extends Camera3D
class_name DebugCamera3D

const MAX_SPEED := 30
const MIN_SPEED := 2
const ACCELERATION := 1
const MOUSE_SENSITIVITY := 0.002

var movement_active := false:
	set(val):
		movement_active = val
		Input.set_mouse_mode(Input.MOUSE_MODE_CAPTURED if movement_active else Input.MOUSE_MODE_VISIBLE)

var target_speed := (MIN_SPEED + MAX_SPEED)/2
var velocity := Vector3.ZERO


func _ready() -> void:
	_add_keybindings()

func _process(delta: float) -> void:
	if movement_active:
		var dir = Vector3.ZERO
		if Input.is_action_pressed("__debug_camera_forward"): 	dir.z -= 1
		if Input.is_action_pressed("__debug_camera_back"): 		dir.z += 1
		if Input.is_action_pressed("__debug_camera_left"): 		dir.x -= 1
		if Input.is_action_pressed("__debug_camera_right"): 	dir.x += 1
		
		dir = dir.normalized()
		dir = self.basis * dir
		
		var speed_mult = 4 if Input.is_action_pressed("__debug_camera_shift") else 1
		velocity = lerp(velocity, dir * target_speed * speed_mult, ACCELERATION)
		self.position += velocity * delta


func _input(event: InputEvent) -> void:
	if movement_active:
		# Turn around
		if event is InputEventMouseMotion:
			#pivot.rotate_y(-event.relative.x * MOUSE_SENSITIVITY)
			rotation.x -= (event.relative.y * MOUSE_SENSITIVITY)
			rotation.y -= (event.relative.x * MOUSE_SENSITIVITY)
			rotation.x = clamp(rotation.x, -PI/2, PI/2)
		
		var speed_up = func():
			target_speed = clamp(target_speed + 1, MIN_SPEED, MAX_SPEED)
			
		var slow_down = func():
			target_speed = clamp(target_speed - 1, MIN_SPEED, MAX_SPEED)
		
		# Speed up and down with the mouse wheel
		if event is InputEventMouseButton:
			if event.button_index == MOUSE_BUTTON_WHEEL_UP and event.pressed:
				speed_up.call()
			if event.button_index == MOUSE_BUTTON_WHEEL_DOWN and event.pressed:
				slow_down.call()


func _add_keybindings() -> void:
	var actions = InputMap.get_actions()
	if "__debug_camera_forward" not in actions: _add_key_input_action("__debug_camera_forward", KEY_W)
	if "__debug_camera_back" 	not in actions: _add_key_input_action("__debug_camera_back", KEY_S)
	if "__debug_camera_left" 	not in actions: _add_key_input_action("__debug_camera_left", KEY_A)
	if "__debug_camera_right" 	not in actions: _add_key_input_action("__debug_camera_right", KEY_D)
	if "__debug_camera_shift" 		not in actions: _add_key_input_action("__debug_camera_shift", KEY_SHIFT)


func _add_key_input_action(name: String, key: Key) -> void:
	var ev = InputEventKey.new()
	ev.physical_keycode = key
	
	InputMap.add_action(name)
	InputMap.action_add_event(name, ev)
