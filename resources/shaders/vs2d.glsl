in vec3 aVertexPosition;
in vec3 aVertexTexCoord;

out vec3 vPosition;
out vec2 vTexCoord;

uniform mat4 uMVPMatrix;
uniform float uPointSize;

void main() {
    vPosition = aVertexPosition;
    vTexCoord = aVertexTexCoord.xy;

    gl_Position = vec4(vPosition.x, vPosition.y, 0, 1);
    gl_PointSize = uPointSize;
}