shader_type spatial;
render_mode blend_mix, unshaded;

uniform vec3 origin = vec3(0.0, 0.0, 0.0);
uniform float min_d : hint_range(0.1, 2, 0.1) = 0.4;
uniform float max_d : hint_range(0.1, 2, 0.1) = 0.8;
uniform vec4 albedo : source_color = vec4(1.0);

void fragment() {
    float d = distance(VERTEX, (VIEW_MATRIX * vec4(origin, 1)).xyz);
    float alpha = clamp(1.0 - smoothstep(min_d, max_d, d), 0.0, 1.0);
	
    ALBEDO = albedo.rgb;
	ALPHA = alpha;
}