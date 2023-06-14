in vec3 aVertexPosition;
in vec3 aColor;
in float aVariance;
in int aGroups;

out vec3 vColor;
out vec4 vPosition;

uniform mat4 uMVPMatrix;
uniform float uVarianceFilter;
uniform int uGroupsFilter;

void main() {
    int noGroup = 0;
    if ((uGroupsFilter & aGroups) == noGroup || aVariance > uVarianceFilter) {
        gl_Position = vec4(-999999, -999999, -999999, 1);
        vColor = vec3(0, 0, 0);
    } else {
        vec3 invertedVertexPosition = vec3(aVertexPosition.x, -aVertexPosition.y, -aVertexPosition.z);
        gl_Position = (uMVPMatrix * vec4(invertedVertexPosition, 1));
        gl_PointSize = 3.0;
        vColor = aColor;
    }
    vPosition = gl_Position;
}
