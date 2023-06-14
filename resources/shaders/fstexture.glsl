in vec2 vTexCoord;
out vec4 fFragColor;

uniform sampler2D uTexture;

void main() {
    fFragColor = texture(uTexture,vTexCoord);
}