shader_type spatial;
uniform float max_d: hint_range(0.0, 200, 0.1) = 10.0;
uniform float min_d: hint_range(0.0, 200, 0.1) = 1.0;
uniform vec3 albedo: source_color;

void fragment() {
	float d = length(CAMERA_POSITION_WORLD - (INV_VIEW_MATRIX * vec4(VERTEX, 1.0)).xyz);
	float t = clamp((d - min_d) / (max_d - min_d), 0.0, 1.0);
	
	vec2 screen_pos = SCREEN_UV * VIEWPORT_SIZE;
	int x = int(screen_pos.x) % 4;
	int y = int(screen_pos.y) % 4;
	
	float bayer[16] = float[](
		0.0/16.0, 8.0/16.0, 2.0/16.0, 10.0/16.0,
		12.0/16.0, 4.0/16.0, 14.0/16.0, 6.0/16.0,
		3.0/16.0, 11.0/16.0, 1.0/16.0, 9.0/16.0,
		15.0/16.0, 7.0/16.0, 13.0/16.0, 5.0/16.0
	);
	
	float dither_value = bayer[y * 4 + x];
	
	if (dither_value < t) {
		discard;
	}
	ALBEDO = albedo;
}