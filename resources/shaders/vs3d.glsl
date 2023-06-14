in vec3 aVertexPosition;
in vec3 aVertexTexCoord;

out vec2 vTexCoord;

uniform mat4 uMVPMatrix;
uniform float uPointSize;

void main() {
    vTexCoord = vec2(aVertexTexCoord.x,-aVertexTexCoord.y);
    vec3 invertedVertexPosition = vec3(aVertexPosition.x, -aVertexPosition.y, -aVertexPosition.z);
    gl_Position = (uMVPMatrix * vec4(invertedVertexPosition, 1));
    gl_PointSize = uPointSize;
}
