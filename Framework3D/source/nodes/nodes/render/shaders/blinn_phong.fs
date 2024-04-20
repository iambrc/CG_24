#version 430 core

// Define a uniform struct for lights
struct Light {
    // The matrices are used for shadow mapping. You need to fill it according to how we are filling it when building the normal maps (node_render_shadow_mapping.cpp). 
    // Now, they are filled with identity matrix. You need to modify C++ code innode_render_deferred_lighting.cpp.
    // Position and color are filled.
    mat4 light_projection;
    mat4 light_view;
    vec3 position;
    float radius;
    vec3 color; // Just use the same diffuse and specular color.
    int shadow_map_id;
};

layout(binding = 0) buffer lightsBuffer {
Light lights[4];
};

uniform vec2 iResolution;

uniform sampler2D diffuseColorSampler;
uniform sampler2D normalMapSampler; // You should apply normal mapping in rasterize_impl.fs
uniform sampler2D metallicRoughnessSampler;
uniform sampler2DArray shadow_maps;
uniform sampler2D position;

// uniform float alpha;
uniform vec3 camPos;

uniform int light_count;

layout(location = 0) out vec4 Color;

void main() {
vec2 uv = gl_FragCoord.xy / iResolution;

vec3 pos = texture2D(position,uv).xyz;
vec3 normal = texture2D(normalMapSampler,uv).xyz;

vec4 metalnessRoughness = texture2D(metallicRoughnessSampler,uv);
float metal = metalnessRoughness.x;
float roughness = metalnessRoughness.y;

vec3 diffusecolor = texture2D(diffuseColorSampler,uv).xyz;
vec3 ambient = vec3(0.2 ,0.2 , 0.2);
Color = vec4(ambient*diffusecolor, 1);

for(int i = 0; i < light_count; i ++) {

vec4 fraglight = lights[i].light_projection * lights[i].light_view * (vec4(pos, 1.0));
vec3 projcoords = fraglight.xyz / fraglight.w;
float depth = projcoords.z;
float closest_depth = texture(shadow_maps, vec3(projcoords.xy*0.5+0.5, lights[i].shadow_map_id)).x;
//float shadow = depth > closest_depth ? 1.0 : 0.0;

// Visualization of shadow map
//Color += vec4(shadow, 0,0,1);
vec3 light_direction = lights[i].position - pos;
vec3 camera_direction = camPos - pos;
vec3 half_direction = normalize(light_direction + camera_direction);

float bias = max(0.05 * (1.0 - dot(normal, normalize(light_direction))), 0.005);
//float shadow = depth - bias > closest_depth ? 1.0 : 0.0;


float shadow = 0.0;
vec2 texelSize = vec2(1.0 / iResolution);
for(int x = -1; x <= 1; ++x)
{
    for(int y = -1; y <= 1; ++y)
    {
        float pcfDepth = texture(shadow_maps, 
            vec3(projcoords.xy*0.5+0.5 + vec2(x, y) * texelSize, lights[i].shadow_map_id)).x; 
        shadow += depth - bias > pcfDepth ? 1.0 : 0.0;        
    }    
}
shadow /= 9.0;

float diffuse = (1-shadow)*(1-0.8*metal)*max(0.f, dot(normal, normalize(light_direction)));
float specular =  (1-shadow)*0.8*metal*max(0.f, dot(normal, half_direction));
//float diffuse = (1-0.8*metal)*max(0.f, dot(normal, normalize(light_direction)));
//float specular =  0.8*metal*max(0.f, dot(normal, half_direction));

if (diffuse == 0.f)
    specular = 0.f;
else
   specular = pow(specular, 8*(1-roughness));

Color += vec4(diffuse*diffusecolor + specular*lights[i].color, 1);

// HW6_TODO: first comment the line above ("Color +=..."). That's for quick Visualization.
// You should first do the Blinn Phong shading here. You can use roughness to modify alpha. Or you can pass in an alpha value through the uniform above.

// After finishing Blinn Phong shading, you can do shadow mapping with the help of the provided shadow_map_value. 
// You will need to refer to the node, node_render_shadow_mapping.cpp, for the light matrices definition. 
// Then you need to fill the mat4 light_projection; mat4 light_view; 
// with similar approach that we fill position and color.
// For shadow mapping, as is discussed in the course, you should compare the value 
// "position depth from the light's view" against the "blocking object's depth.", 
// then you can decide whether it's shadowed.

// PCSS is also applied here.
}

Color = pow(Color,vec4(1.0/2.2));
}