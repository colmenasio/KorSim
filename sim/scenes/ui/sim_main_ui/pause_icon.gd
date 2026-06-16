extends Control

const blink_period: float = 0.75
var _acc: float = 0

func _ready() -> void:
	self.visibility_changed.connect(func(): self._acc = 0; $Panel.visible = true; $Panel2.visible = true)

func _process(delta: float) -> void:
	_acc += delta
	if _acc >= blink_period:
		$Panel.visible = not $Panel.visible
		$Panel2.visible = not $Panel2.visible
		_acc = 0
