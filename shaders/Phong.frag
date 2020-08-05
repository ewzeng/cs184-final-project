#version 330

uniform vec4 u_color;
uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

in vec4 v_position;
in vec4 v_normal;
in vec2 v_uv;

out vec4 out_color;

void main() {
  // YOUR CODE HERE
  vec3 ambient = vec3(0.1, 0.1, 0.1);

  vec3 ray = u_light_pos - v_position.xyz;
  float m = max(0.0, dot(normalize(ray), normalize(v_normal.xyz)));
  vec3 diffuse = (u_light_intensity / dot(ray, ray)) * m;

  vec3 h = normalize(ray + u_cam_pos - v_position.xyz);
  float m2 = max(0.0, dot(h, normalize(v_normal.xyz)));
  m2 = pow(m2, 64);
  vec3 specular = (u_light_intensity / dot(ray, ray)) * m2;
 
  out_color = vec4(ambient + diffuse + specular, 1.0);
}

