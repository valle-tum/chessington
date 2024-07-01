extends Node

const DISPLAY_IMG_RBG = 0;
const DISPLAY_IMG_GREY = 1;
const DISPLAY_IMG_PIXELSTRIP = 2;
const DISPLAY_MARKER = 3;

#var size = Vector2(640, 480);

onready var camera = preload("res://camera.gdns").new()

#var active = false
#var player
var camera_size
var texture_size
var image = Image.new();
var display_option = DISPLAY_IMG_RBG;

signal camera_ready
signal camera_frame

signal game_start
signal force

func _ready():
#	camera.set_default(0);
	camera.open();
	camera.flip(true, false);
	camera_size = Vector2(camera.get_width(), camera.get_height());
	texture_size = max(camera_size.x, camera_size.y);
	print("texture size: ", texture_size);
	
	emit_signal("camera_ready", camera_size);

	#player = get_node("/root/root/game/player");

	#$animation.play("greeting_blink");

func _process(delta):
	var buffer = camera.get_image(display_option);
		
	if not buffer:
		return;
	
	#var first_strip_height = camera.get_first_strip_height();
	#print("result of get_first_strip_height(): ", first_strip_height);
	image.create_from_data(texture_size, texture_size, false, Image.FORMAT_RGB8, buffer);
	emit_signal("camera_frame", image)
	
	#if !active:
	#	var face = camera.detect_face()
	#	if face and not active:
	#		var position = face.position + face.size / 2;
	#		if (position - size / 2).length() < 40:
	#			active = true
	#			emit_signal("game_start", position)
	#			$animation.play("game_start")
	#else:
	#	var region = Rect2(player.get("position"), Vector2(70, 70))
	#	var force = camera.compute_flow(region)
	#	emit_signal("force", Vector2(clamp(force.x, -100, 100), clamp(force.y, -100, 100)))

func _on_HSlider_threshold_value_changed(value):
	print("threshold: ", value)
	camera.set_threshold(value)

func _on_ItemList_item_selected_ImageDisplay(option):
	print("output image option: ", option);
	display_option = option;
