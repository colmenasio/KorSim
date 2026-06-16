@tool
extends TextureRect
class_name GyroCompass

@export var text_size: Vector2i = Vector2(512, 512):
	set(val):
		$SubViewport.size = val
		text_size = $SubViewport.size

func _ready() -> void:
	text_size = $SubViewport.size

func set_compass_rotation(rot: Vector3):
	$SubViewport/Compass.rotation = rot
