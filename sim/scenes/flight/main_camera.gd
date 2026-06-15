@tool extends Camera3D

@export_range(0.2, 10, 0.1) var delay_constant: float = 0.4
@export var snap: bool = false:
	set(value):
		self.top_level = !value
		snap = value

func _process(delta: float) -> void:
	if(!snap):
		var mu_p: Vector3 = get_parent().global_position - self.global_position
		# GOD I LOVE MANIFORDS I LOVE DIFFERENTIABLE MANIFOLDS I LOVE LIE GROUPS AND I LOVE SO(2)
		var pr = get_parent().global_rotation
		var sr = self.global_rotation
		var mu_r_p = Vector3(fmod(pr.x, PI), fmod(pr.y, PI), fmod(pr.z, PI))
		var mu_r_s = Vector3(fmod(sr.x, PI), fmod(sr.y, PI), fmod(sr.z, PI))
		var mu_r = pr - sr # [-2PI - 2PI ]
		
		if mu_r.x > PI:
			mu_r.x -= 2*PI
		if mu_r.x < -PI:
			mu_r.x += 2*PI
		if mu_r.y > PI:
			mu_r.y -= 2*PI
		if mu_r.y < -PI:
			mu_r.y += 2*PI
		if mu_r.z > PI:
			mu_r.z -= 2*PI
		if mu_r.z < -PI:
			mu_r.z += 2*PI
		
		self.global_position += mu_p * delta / delay_constant
		self.global_rotation += mu_r * delta / delay_constant
