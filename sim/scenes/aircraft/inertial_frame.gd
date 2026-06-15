extends Node3D

func _process(delta: float) -> void:
	update_visual_gizmos()

func update_visual_gizmos():
	$MeshInstance3D.position = $Aircraft.position
	
	var grav_force_vector = $Aircraft._model.get_gravity_force()/10000
	var grav_mag = grav_force_vector.length()
	if grav_mag < 0.01:
		$GravForceVector.visible = false
	else:
		$GravForceVector.visible = true
		$GravForceVector.position = $Aircraft.position + Quaternion.from_euler($Aircraft.rotation) * grav_force_vector / 2
		return
		var target = $GravForceVector.position - $Aircraft.position
		if(target.normalized().cross(Vector3.UP).length() < 0.01):
			$GravForceVector.rotation = Vector3(PI/2, 0, 0)
		else:
			$GravForceVector.look_at(
				Vector3.ZERO,
				Vector3.UP
			)
		$GravForceVector.scale.z = grav_mag
