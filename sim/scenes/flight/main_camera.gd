extends Camera3D

@export_range(0.05, 1, 0.1) var delay_constant: float = 0.4
@export var snap: bool = false:
	set(value):
		self.top_level = !value
		snap = value
#@export_enum("inertial_frame:true", "body_frame:false") var align_rotation

func _process(delta: float) -> void:
	# El error no deberia ser lineal sino slerpeado!!
	if(!snap):
		var mu_p: Vector3 = get_parent().global_position - self.global_position
		# GOD I LOVE MANIFORDS I LOVE DIFFERENTIABLE MANIFOLDS I LOVE LIE GROUPS AND I LOVE SO(2)
		var pr = get_parent().global_rotation
		var sr = self.global_rotation
		var mu_r_p = Vector3(fmod(pr.x, PI), fmod(pr.y, PI), fmod(pr.z, PI))
		var mu_r_s = Vector3(fmod(sr.x, PI), fmod(sr.y, PI), fmod(sr.z, PI))
		var mu_r = mu_r_p - mu_r_s # [-2PI - 2PI ]
		
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
