// Inputs from vertex stage
in vec3 v_Color;
in vec3 v_WorldPosition;

// Frag output
out vec4 FragColor;

void main()
{
    vec3 normal = normalize(cross(dFdx(v_WorldPosition), dFdy(v_WorldPosition)));
    vec3 lightPosition = vec3(3.5, 8.0, 5.0);
    vec3 toLight = lightPosition - v_WorldPosition;
    float distanceToLight = length(toLight);
    vec3 lightDirection = toLight / max(distanceToLight, 0.001);
    float diffuse = max(dot(normal, lightDirection), 0.0);
    float attenuation = 1.0 / (1.0 + 0.09 * distanceToLight + 0.032 * distanceToLight * distanceToLight);
    vec3 lighting = v_Color * (0.16 + 4.0 * diffuse * attenuation);
    FragColor = vec4(lighting, 1.0);
}