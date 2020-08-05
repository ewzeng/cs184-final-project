#version 330

uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

uniform vec4 u_color;

uniform sampler2D u_texture_2;
uniform vec2 u_texture_2_size;

uniform float u_normal_scaling;
uniform float u_height_scaling;

in vec4 v_position;
in vec4 v_normal;
in vec4 v_tangent;
in vec2 v_uv;

out vec4 out_color;

float h(vec2 uv) {
  return texture(u_texture_2, uv).r;
}

void main() {
  // YOUR CODE HERE
  vec3 b = cross(v_normal.xyz, v_tangent.xyz);
  mat3 TBN = mat3(v_tangent.xyz, b, v_normal.xyz);

  float ww = u_texture_2_size.x;
  float hh = u_texture_2_size.y;
  
  float dU = (h(vec2(v_uv.x + 1.0 / ww, v_uv.y)) - h(v_uv)) * u_height_scaling * u_normal_scaling;
  float dV = (h(vec2(v_uv.x, v_uv.y + 1.0 / hh)) - h(v_uv)) * u_height_scaling * u_normal_scaling;

  vec3 nn = TBN * vec3(-dU, -dV, 1.0);
  nn = normalize(nn);

  // Phong shading
  vec3 ambient = vec3(0.05, 0.05, 0.05);

  vec3 ray = u_light_pos - v_position.xyz;
  float m = max(0.0, dot(normalize(ray), nn));
  vec3 diffuse = (u_light_intensity / dot(ray, ray)) * m;

  vec3 halfv = normalize(ray + u_cam_pos - v_position.xyz);
  float m2 = max(0.0, dot(halfv, nn));
  m2 = pow(m2, 64);
  vec3 specular = (u_light_intensity / dot(ray, ray)) * m2;
 
  out_color = vec4(ambient + diffuse + specular, 1.0);
}

