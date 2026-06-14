extends Node

static var flight_scene: PackedScene = preload("res://scenes/flight/flight.tscn")

var main: Main 

var curr_scene: Node

func initialize(main_scene: Main) -> void:
	main = main_scene
	curr_scene = null

func load_flight_scene() -> void:
	unload_current_scene()
	curr_scene = flight_scene.instantiate()
	main.add_child(curr_scene)

func unload_current_scene():
	if curr_scene != null:
		main.remove_child(curr_scene)
		curr_scene.queue_free()
