extends Node

static var flight_scene: PackedScene = preload("res://scenes/flight/flight.tscn")
static var main_ui_scene : PackedScene = preload("res://scenes/ui/sim_main_ui/sim_main_ui.tscn") 

var _main: Main 

var _main_ui: SimMainUI
var _curr_scene: Node

func initialize(main_scene: Main) -> void:
	_main = main_scene
	_main_ui = main_ui_scene.instantiate()
	_main.add_child(_main_ui)
	_curr_scene = null

func load_flight_scene() -> void:
	unload_current_scene()
	_curr_scene = flight_scene.instantiate()
	_main.add_child(_curr_scene)

func unload_current_scene():
	if _curr_scene != null:
		_main.remove_child(_curr_scene)
		_curr_scene.queue_free()

func get_main_ui() -> SimMainUI:
	return _main_ui

func get_active_scene() -> Node:
	return _curr_scene
