extends Node3D

func _process(_delta: float) -> void:
	update_visual_gizmos()

func update_visual_gizmos():
	$VerticalLineGizmo.position = $Aircraft.position
	
	var model: FESixDofModel = $Aircraft._model
	
	var grav_force_vector = model.get_gravity_force()/10000
	var aero_force_vector = model.get_aerodynamic_force()/10000
	var drag_force_vector = model.get_drag_force()/10000
	var lift_force_vector = model.get_lift_force()/10000
	var engine_force_vector = model.get_engine_force()/10000
	var side_force_vector = model.get_side_force()/10000
	
	update_vector_gizmo($ForcesGizmos/FG, grav_force_vector)
	update_vector_gizmo($ForcesGizmos/FA, aero_force_vector)
	update_vector_gizmo($ForcesGizmos/FD, drag_force_vector)
	update_vector_gizmo($ForcesGizmos/FL, lift_force_vector)
	update_vector_gizmo($ForcesGizmos/FE, engine_force_vector)
	update_vector_gizmo($ForcesGizmos/FY, side_force_vector)


func update_vector_gizmo(node: Node3D, vector: Vector3):
	var mag = vector.length()
	node.scale.z = mag
	
	if mag < 0.01:
		node.visible = false
		return

	node.visible = true 
	var p_pos : Vector3 = $Aircraft.position + Quaternion.from_euler($Aircraft.rotation) * vector / 2
	var p_target :Vector3 = $Aircraft.position
	
	var forward :Vector3 = p_target - p_pos
	if(forward.normalized().cross(Vector3.UP).is_zero_approx()):
		node.position = p_pos
		node.rotation = Vector3(PI/2, 0, 0)
		return
	
	var lookat_basis = Basis.looking_at(forward, Vector3.UP);
	var original_scale = node.get_scale();
	node.set_global_transform(Transform3D(lookat_basis, p_pos));
	node.set_scale(original_scale);
