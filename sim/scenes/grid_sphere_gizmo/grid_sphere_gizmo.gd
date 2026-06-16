@tool
extends Node3D

var _gizmo_radius: float = 180
var _spheres_radius: float = 1
var _resolution: float = 60
var _sphere_color: Color = Color.CRIMSON

func _ready() -> void:
	self._generate_ball_grid()

func _process(_delta: float) -> void:
	$Anchor.global_position = (self.global_position / _resolution).round()*_resolution

func _generate_ball_grid() -> void:
	var spheres_per_axis = int(_gizmo_radius / _resolution) * 2 + 1
	@warning_ignore("integer_division") var center = spheres_per_axis / 2
	
	var total_spheres = spheres_per_axis * spheres_per_axis * spheres_per_axis
	if total_spheres > 1000:
		push_error("Multimeshes are optimized and all but too many reference balls are too many")
	
	var multimesh: MultiMesh = MultiMesh.new()
	multimesh.transform_format = MultiMesh.TRANSFORM_3D
	multimesh.mesh = SphereMesh.new()
	multimesh.instance_count = total_spheres
	
	# Configure sphere mesh
	var mesh: SphereMesh = multimesh.mesh
	mesh.radius = _spheres_radius
	mesh.height = _spheres_radius * 2
	#mesh.material = StandardMaterial3D.new()
	#mesh.material.albedo = _sphere_color
	mesh.material = ShaderMaterial.new()
	mesh.material.shader = preload("res://scenes/aircraft/sphere_gizmo.gdshader")
	mesh.material.set_shader_parameter("albedo", _sphere_color)
	mesh.material.set_shader_parameter("min_d", _gizmo_radius * 0.80)
	mesh.material.set_shader_parameter("max_d", _gizmo_radius * 0.95)
	
	var index = 0
	for i in spheres_per_axis:
		for j in spheres_per_axis:
			for k in spheres_per_axis:
				# Calculate position relative to center
				var x = (i - center) * _resolution
				var y = (j - center) * _resolution
				var z = (k - center) * _resolution
				
				var tf = Transform3D.IDENTITY
				tf.origin = Vector3(x, y, z)
				multimesh.set_instance_transform(index, tf)
				index += 1
	
	# Update actual instance count (some may have been skipped)
	multimesh.instance_count = index
	$Anchor/Spheres.multimesh = multimesh
