R"luastring(

#define MAX_LIGHTS 16
#define NULL_LIGHT MAX_LIGHTS      // slot reservado, sempre cor (0,0,0)
#define LIGHT_SLOTS (MAX_LIGHTS + 1)

uniform mat4 projectionMatrix;
uniform mat4 viewMatrix;
uniform bool isCanvasEnabled;

uniform vec4 globalLight;
uniform vec3 sunDirToLight;

uniform float fogStart;
uniform float fogEnd;
uniform vec4 fogColor;

varying vec3 viewPosition;
varying vec3 vertexNormal;
varying vec4 vertexColor;
varying vec4 instanceColor;

// Bayer 4x4 ordered-dither threshold in [0, 15/16].
// Pure arithmetic (no switch / no 16-way branch). Same matrix as before:
//   0  8  2 10
//  12  4 14  6
//   3 11  1  9
//  15  7 13  5
float bayer4x4(vec2 p) {
	vec2 q = mod(floor(p), 4.0);      // mod() is floored => always 0..3, even for negative input
	vec2 lo = mod(q, 2.0);            // low bits  (x0, y0)
	vec2 hi = floor(q * 0.5);         // high bits (x1, y1)
	return (8.0 * abs(lo.x - lo.y) + 4.0 * lo.y + 2.0 * abs(hi.x - hi.y) + hi.y) * (1.0 / 16.0);
}

#ifdef VERTEX

layout(location = 3) in vec3 vertNormal;

layout(location = 4) in vec4 InstanceMat1;
layout(location = 5) in vec4 InstanceMat2;
layout(location = 6) in vec4 InstanceMat3;
layout(location = 7) in vec4 InstanceMat4;
layout(location = 8) in vec4 InstanceColor;

vec4 position(mat4 transformProjection, vec4 vertexPosition) {
	instanceColor = InstanceColor;
	// InstanceMatN are the ROWS of the model matrix: dot products replace building a mat4 per vertex
	vec4 worldPos = vec4(dot(InstanceMat1, vertexPosition),
	                     dot(InstanceMat2, vertexPosition),
	                     dot(InstanceMat3, vertexPosition),
	                     dot(InstanceMat4, vertexPosition));

	vec4 viewPos = viewMatrix * worldPos;
	viewPosition = viewPos.xyz;

	vertexNormal = normalize(vec3(dot(InstanceMat1.xyz, vertNormal),
	                              dot(InstanceMat2.xyz, vertNormal),
	                              dot(InstanceMat3.xyz, vertNormal)));
	vertexColor = VertexColor;

	vec4 screenPosition = projectionMatrix * viewPos;

	if(isCanvasEnabled)
		screenPosition.y *= -1.0;

	return screenPosition;
}

#endif

#ifdef PIXEL

vec4 effect(vec4 color, Image tex, vec2 texcoord, vec2 pixcoord) {
	vec4 texcolor = Texel(tex, vec2(texcoord.x, 1.0 - texcoord.y));

	if(texcolor.a == 0.0)
		discard;

	float alpha = color.a * vertexColor.a * globalLight.a * instanceColor.a;

	if(alpha <= 0.0)
		discard;

	// the dither threshold never exceeds 15/16, so fully opaque pixels skip it entirely
	if(alpha < 0.9375 && alpha < bayer4x4(pixcoord))
		discard;

	vec4 baseColor = texcolor * color * vertexColor * instanceColor;

	vec3 N = normalize(vertexNormal);
	float ad = 0.8 + max(dot(N, sunDirToLight), 0.0);
	baseColor *= vec4(vec3(ad), 1.0) * globalLight;

	float fogFactor = clamp((length(viewPosition) - fogStart) / (fogEnd - fogStart), 0.0, 1.0);

	return mix(baseColor, fogColor, fogFactor);
}

#endif
)luastring"