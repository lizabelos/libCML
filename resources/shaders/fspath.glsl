out vec4 fFragColor;

in vec3 vColor;
in vec4 vPosition;

void main() {
    fFragColor = vec4(vColor, 1);
}