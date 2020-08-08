#version 330 core
out vec4 out_color;

void main()
{
    vec2 circCoord = 2.0 * gl_PointCoord - 1.0;
    float m = dot(circCoord, circCoord);
    if (m > 1.0) {
        discard;
    }
    
    if (m > 0.8) {
        out_color = vec4(0.0f, 0.0f, 0.0f, 1.0f);
    } else if (m > 0.03) {
        out_color = vec4(3.0f, 0.5f, 0.2f, 1.0f);
    } else {
        out_color = vec4(6.0f, 1.0f, 0.4f, 1.0f);
    }
}
