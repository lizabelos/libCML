in vec3 aVertexPosition;

out vec3 vColor;
out vec4 vPosition;

uniform mat4 uMVPMatrix;

void main() {
    vec3 invertedVertexPosition = vec3(aVertexPosition.x, -aVertexPosition.y, -aVertexPosition.z);
    gl_Position = (uMVPMatrix * vec4(invertedVertexPosition, 1));
    vColor = vec3(1, 1, 1);
    vPosition = gl_Position;
}
