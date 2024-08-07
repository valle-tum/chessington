extends Node

const DISPLAY_IMG_RBG = 0;


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
#	camera.set_resolution(Vector2(1920, 1080))
	camera_size = Vector2(camera.get_width(), camera.get_height());
	texture_size = max(camera_size.x, camera_size.y);
	print("texture size: ", texture_size);
	
	emit_signal("camera_ready", camera_size);


func _process(delta):
	var buffer = camera.get_image(display_option);
		
	if not buffer:
		return;
	
	#var first_strip_height = camera.get_first_strip_height();
	#print("result of get_first_strip_height(): ", first_strip_height);
	image.create_from_data(texture_size, texture_size, false, Image.FORMAT_RGB8, buffer);
	emit_signal("camera_frame", image)
	
#	print(camera.get_counterFrame())
	$TextureProgress.value = camera.get_counterFrame()
	

func _on_HSlider_threshold_value_changed(value):
	print("threshold: ", value)
	camera.set_threshold(value)

func _on_ItemList_item_selected_ImageDisplay(option):
	print("output image option: ", option);
	display_option = option;


func _on_ProgressBar_gui_input(event):
	pass # Replace with function body.
