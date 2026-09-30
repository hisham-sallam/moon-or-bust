#version 330

in vec3 fragPosition;
in vec2 fragTexCoord;
in vec4 fragColor;
in vec3 fragNormal;

uniform sampler2D texture0;
uniform vec4 colDiffuse;

out vec4 finalColor;

void main()
{
    vec3 albedo = vec3(0.75, 0.85, 1.0);

    finalColor = vec4(albedo, 0.10);
}