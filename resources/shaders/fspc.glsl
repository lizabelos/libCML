out vec4 fFragColor;

in vec3 vColor;
in vec4 vPosition;

void main() {
    // float far = 10.0;
    // float near = 1.0;
    // float ndcDepth = (2.0 * vPosition.z - near - far) / (far - near);
    // float clipDepth = clamp(ndcDepth * 0.5 / vPosition.w + 0.5, 0.0, 1.0);
    // fFragColor = vec4(clipDepth, 0.0, 1.0 - clipDepth, 1.0);
    fFragColor = vec4(vColor, 1);
}