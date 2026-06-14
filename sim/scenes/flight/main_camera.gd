@tool extends Camera3D

@export_range(0.2, 10, 0.1) var delay_constant: float = 0.4

func _process(delta: float) -> void:
	var mu_p: Vector3 = get_parent().global_position - self.global_position
	var mu_rot: Vector3 = get_parent().rotation - self.rotation
	mu_rot.x = fmod(mu_rot, 360)
	
	self.global_position += mu_p * delta / delay_constant
