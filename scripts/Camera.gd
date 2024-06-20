extends Sprite

var texture_size
var image = Image.new()

func _on_root_camera_frame(new_frame):
	texture.set_data(new_frame);

func _on_root_camera_ready(size):
	print("camera ready - size: ", size);
	texture_size = max(size.x, size.y);
	texture = ImageTexture.new();
	texture.create(texture_size, texture_size, Image.FORMAT_RGB8, 0);
	set("region_rect", Rect2(0, 0, size.x, size.y));
	set_texture(texture);
