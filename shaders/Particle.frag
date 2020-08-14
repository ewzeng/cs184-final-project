#version 330 core
out vec4 out_color;

void main()
{
    vec2 circCoord = 2.0 * gl_PointCoord - 1.0;
    float m = dot(circCoord, circCoord);
    if (m > 1.0) {
        discard;
    }

    out_color = (1.0 - m) * vec4(0.0f, 0.0f, 0.8f, 1.0f);
}
