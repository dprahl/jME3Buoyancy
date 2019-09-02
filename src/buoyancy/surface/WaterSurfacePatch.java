package buoyancy.surface;

import com.jme3.math.Matrix4f;
import com.jme3.math.Vector2f;
import com.jme3.math.Vector3f;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.VertexBuffer.Type;
import com.jme3.util.BufferUtils;

/**
 *
 * @author User
 */
public class WaterSurfacePatch extends Mesh {
    private Vector3f[] vertices;
    private Vector3f[] normals;
    private Vector3f[] tangents;
    private Vector3f[] binormals;
    private Vector2f[] texCoords;
    private int[] indices;
    private int pointsX;
    private int pointsZ;
    private float scale;
    private Matrix4f localToWorld;
    private Vector3f tempWorldLoc;
    
    @Deprecated
    private WaterSurfacePatch() {
        initHardcoded();
    }
    
    public WaterSurfacePatch(int pointsX, int pointsZ, float scale) {
        init(pointsX, pointsZ, scale);
    }
    
    public WaterSurfacePatch(int pointsX, int pointsZ) {
        this(pointsX, pointsZ, 1f);
    }
    
    private void init(int pointsX, int pointsZ, float scale) {
        this.pointsX = pointsX;
        this.pointsZ = pointsZ;
        this.scale = scale;
        this.localToWorld = new Matrix4f();
        this.tempWorldLoc = new Vector3f();
        
        // initialize arrays
        int totalPoints = pointsX * pointsZ;
        vertices = new Vector3f[totalPoints];
        normals = new Vector3f[totalPoints];
        tangents = new Vector3f[totalPoints];
        binormals = new Vector3f[totalPoints];
        texCoords = new Vector2f[totalPoints];
        indices = new int[(pointsX-1) * (pointsZ-1) * 6];
        
        // vertex positions (all initialized to y = 0)
        for(int z=0; z <pointsZ; z++) {
            for(int x=0; x <pointsX; x++) {
                vertices[z * pointsX + x] = new Vector3f(x*scale, 0f, z*scale);
            }
        }
        
        // triangle indices - counter-clockwise ordering to face straight up (+Y)
        int currentIndex = 0;
        for(int z=0; z < pointsZ-1; z++) {
            for(int x=0; x < pointsX-1; x++) {
                // 1 cell = 2 triangles
                int indexOffset = z * pointsX + x;
                // triangle 1
                indices[currentIndex++] = indexOffset;
                indices[currentIndex++] = indexOffset + pointsX;
                indices[currentIndex++] = indexOffset + 1;
                // triangle 2
                indices[currentIndex++] = indexOffset + 1;
                indices[currentIndex++] = indexOffset + pointsX;
                indices[currentIndex++] = indexOffset + pointsX + 1;
            }
        }
        
        // texture coordinates, texture will stretch to cover patch
        for(int z =0; z <pointsZ; z++) {
            for(int x =0; x <pointsX; x++) {
                texCoords[z * pointsX + x] = new Vector2f(x/(pointsX -1f), z/(pointsZ -1f));
            }
        }
        
        // normals, all initialized to straight up (+y)
        for(int i =0; i < normals.length; i++) {
            normals[i] = Vector3f.UNIT_Y.clone();
        }
        
        // tangents, all default to straight forward (+z)
        for(int i =0; i < tangents.length; i++) {
            tangents[i] = Vector3f.UNIT_Z.clone();
        }
        
        // binromals, all default to straight right (+x)
        for(int i =0; i < binormals.length; i++) {
            binormals[i] = Vector3f.UNIT_X.clone();
        }
        
        this.setBuffer(Type.Position, 3, BufferUtils.createFloatBuffer(vertices));
        this.setBuffer(Type.Normal, 3, BufferUtils.createFloatBuffer(normals));
        this.setBuffer(Type.Tangent, 3, BufferUtils.createFloatBuffer(tangents));
        this.setBuffer(Type.Binormal, 3, BufferUtils.createFloatBuffer(binormals));
        this.setBuffer(Type.TexCoord, 2, BufferUtils.createFloatBuffer(texCoords));
        this.setBuffer(Type.Index, 1, BufferUtils.createIntBuffer(indices));
        
        //this.setDynamic();// unnecessary, not adding or removing vertices
        this.updateBound();
    }
    
    @Deprecated
    private void initHardcoded() {
        // HARDCODED (no scaling)
        // create a regular horizontal grid of 5 * 5 cells with side length 1
        // origin at top left corner
        // 6 verts across * 6 verts down = 36 verts
        // 25 cells * 2 triangles each = 50 tris * 3 indices = 150 total indices
        
        // initialize arrays
        vertices = new Vector3f[36];
        normals = new Vector3f[36];
         tangents = new Vector3f[36];
        texCoords = new Vector2f[36];
        indices = new int[150];
        
        // vertex positions (x across, then z down, all at y = 0)
        vertices[ 0] = new Vector3f(0, 0, 0);
        vertices[ 1] = new Vector3f(1, 0, 0);
        vertices[ 2] = new Vector3f(2, 0, 0);
        vertices[ 3] = new Vector3f(3, 0, 0);
        vertices[ 4] = new Vector3f(4, 0, 0);
        vertices[ 5] = new Vector3f(5, 0, 0);
        //
        vertices[ 6] = new Vector3f(0, 0, 1);
        vertices[ 7] = new Vector3f(1, 0, 1);
        vertices[ 8] = new Vector3f(2, 0, 1);
        vertices[ 9] = new Vector3f(3, 0, 1);
        vertices[10] = new Vector3f(4, 0, 1);
        vertices[11] = new Vector3f(5, 0, 1);
        //
        vertices[12] = new Vector3f(0, 0, 2);
        vertices[13] = new Vector3f(1, 0, 2);
        vertices[14] = new Vector3f(2, 0, 2);
        vertices[15] = new Vector3f(3, 0, 2);
        vertices[16] = new Vector3f(4, 0, 2);
        vertices[17] = new Vector3f(5, 0, 2);
        //
        vertices[18] = new Vector3f(0, 0, 3);
        vertices[19] = new Vector3f(1, 0, 3);
        vertices[20] = new Vector3f(2, 0, 3);
        vertices[21] = new Vector3f(3, 0, 3);
        vertices[22] = new Vector3f(4, 0, 3);
        vertices[23] = new Vector3f(5, 0, 3);
        //
        vertices[24] = new Vector3f(0, 0, 4);
        vertices[25] = new Vector3f(1, 0, 4);
        vertices[26] = new Vector3f(2, 0, 4);
        vertices[27] = new Vector3f(3, 0, 4);
        vertices[28] = new Vector3f(4, 0, 4);
        vertices[29] = new Vector3f(5, 0, 4);
        //
        vertices[30] = new Vector3f(0, 0, 5);
        vertices[31] = new Vector3f(1, 0, 5);
        vertices[32] = new Vector3f(2, 0, 5);
        vertices[33] = new Vector3f(3, 0, 5);
        vertices[34] = new Vector3f(4, 0, 5);
        vertices[35] = new Vector3f(5, 0, 5);
        
        // triangle indices - counter-clockwise ordering to face up (+y)
        // cell 1
        indices[  0] = 0;
        indices[  1] = 6;
        indices[  2] = 1;
            indices[  3] = 1;
            indices[  4] = 6;
            indices[  5] = 7;
        // cell 2
        indices[  6] = 1;
        indices[  7] = 7;
        indices[  8] = 2;
            indices[  9] = 2;
            indices[ 10] = 7;
            indices[ 11] = 8;
        // cell 3
        indices[ 12] = 2;
        indices[ 13] = 8;
        indices[ 14] = 3;
            indices[ 15] = 3;
            indices[ 16] = 8;
            indices[ 17] = 9;
        // cell 4
        indices[ 18] = 3;
        indices[ 19] = 9;
        indices[ 20] = 4;
            indices[ 21] = 4;
            indices[ 22] = 9;
            indices[ 23] = 10;
        // cell 5
        indices[ 24] = 4;
        indices[ 25] = 10;
        indices[ 26] = 5;
            indices[ 27] = 5;
            indices[ 28] = 10;
            indices[ 29] = 11;
        // cell 6
        indices[ 30] = 6;
        indices[ 31] = 12;
        indices[ 32] = 7;
            indices[ 33] = 7;
            indices[ 34] = 12;
            indices[ 35] = 13;
        // cell 7
        indices[ 36] = 7;
        indices[ 37] = 13;
        indices[ 38] = 8;
            indices[ 39] = 8;
            indices[ 40] = 13;
            indices[ 41] = 14;
        // cell 8
        indices[ 42] = 8;
        indices[ 43] = 14;
        indices[ 44] = 9;
            indices[ 45] = 9;
            indices[ 46] = 14;
            indices[ 47] = 15;
        // cell 9
        indices[ 48] = 9;
        indices[ 49] = 15;
        indices[ 50] = 10;
            indices[ 51] = 10;
            indices[ 52] = 15;
            indices[ 53] = 16;
        // cell 10
        indices[ 54] = 10;
        indices[ 55] = 16;
        indices[ 56] = 11;
            indices[ 57] = 11;
            indices[ 58] = 16;
            indices[ 59] = 17;
        // cell 11
        indices[ 60] = 12;
        indices[ 61] = 18;
        indices[ 62] = 13;
            indices[ 63] = 13;
            indices[ 64] = 18;
            indices[ 65] = 19;
        // cell 12
        indices[ 66] = 13;
        indices[ 67] = 19;
        indices[ 68] = 14;
            indices[ 69] = 14;
            indices[ 70] = 19;
            indices[ 71] = 20;
        // cell 13
        indices[ 72] = 14;
        indices[ 73] = 20;
        indices[ 74] = 15;
            indices[ 75] = 15;
            indices[ 76] = 20;
            indices[ 77] = 21;
        // cell 14
        indices[ 78] = 15;
        indices[ 79] = 21;
        indices[ 80] = 16;
            indices[ 81] = 16;
            indices[ 82] = 21;
            indices[ 83] = 22;
        // cell 15
        indices[ 84] = 16;
        indices[ 85] = 22;
        indices[ 86] = 17;
            indices[ 87] = 17;
            indices[ 88] = 22;
            indices[ 89] = 23;
        // cell 16
        indices[ 90] = 18;
        indices[ 91] = 24;
        indices[ 92] = 19;
            indices[ 93] = 19;
            indices[ 94] = 24;
            indices[ 95] = 25;
        // cell 17
        indices[ 96] = 19;
        indices[ 97] = 25;
        indices[ 98] = 20;
            indices[ 99] = 20;
            indices[100] = 25;
            indices[101] = 26;
        // cell 18
        indices[102] = 20;
        indices[103] = 26;
        indices[104] = 21;
            indices[105] = 21;
            indices[106] = 26;
            indices[107] = 27;
        // cell 19
        indices[108] = 21;
        indices[109] = 27;
        indices[110] = 22;
            indices[111] = 22;
            indices[112] = 27;
            indices[113] = 28;
        // cell 20
        indices[114] = 22;
        indices[115] = 28;
        indices[116] = 23;
            indices[117] = 23;
            indices[118] = 28;
            indices[119] = 29;
        // cell 21
        indices[120] = 24;
        indices[121] = 30;
        indices[122] = 25;
            indices[123] = 25;
            indices[124] = 30;
            indices[125] = 31;
        // cell 22
        indices[126] = 25;
        indices[127] = 31;
        indices[128] = 26;
            indices[129] = 26;
            indices[130] = 31;
            indices[131] = 32;
        // cell 23
        indices[132] = 26;
        indices[133] = 32;
        indices[134] = 27;
            indices[135] = 27;
            indices[136] = 32;
            indices[137] = 33;
        // cell 24
        indices[138] = 27;
        indices[139] = 33;
        indices[140] = 28;
            indices[141] = 28;
            indices[142] = 33;
            indices[143] = 34;
        // cell 25
        indices[144] = 28;
        indices[145] = 34;
        indices[146] = 29;
            indices[147] = 29;
            indices[148] = 34;
            indices[149] = 35;
        
        // texture coordinates
        for(int x =0; x <6; x++) {
            for(int z =0; z <6; z++) {
                texCoords[x * 6 + z] = new Vector2f((float)z/(6 -1), (float)x/(6 -1)); // flipped
            }
        }
        
        // normals, initialized to all straight up (+y)
        for(int i =0; i < normals.length; i++) {
            normals[i] = Vector3f.UNIT_Y.clone();
        }
        
        // tangents, initialized to all stright forward (+z)
        for(int i =0; i < tangents.length; i++) {
            tangents[i] = Vector3f.UNIT_Z.clone();
        }
        
        this.setBuffer(Type.Position, 3, BufferUtils.createFloatBuffer(vertices));
        this.setBuffer(Type.Normal, 3, BufferUtils.createFloatBuffer(normals));
        this.setBuffer(Type.Tangent, 3, BufferUtils.createFloatBuffer(tangents));
        this.setBuffer(Type.TexCoord, 2, BufferUtils.createFloatBuffer(texCoords));
        this.setBuffer(Type.Index, 1, BufferUtils.createIntBuffer(indices));
        
        this.setDynamic();
        this.updateBound();
    }
    
    public void updateHeights(float elapsedTime, Geometry geom) {
        // set localToWorld matrix representing geom world transform for this frame
        geom.getLocalToWorldMatrix(localToWorld);
        
        for(int z=0; z <pointsZ; z++) {
            for(int x=0; x <pointsX; x++) {
                // set tempWorldLoc to current vertex transformed into world coordinates
                localToWorld.mult(vertices[z * pointsX + x], tempWorldLoc);
                
                vertices[z * pointsX + x].setY(WaterSurfaceGenerator.
                        getHeightAt(x*scale, z*scale, elapsedTime));
                normals[z * pointsX + x] = WaterSurfaceGenerator.
                            getNormalAt(x*scale, z*scale, elapsedTime);
                tangents[z * pointsX + x] = WaterSurfaceGenerator.
                            getTangentAt(x*scale, z*scale, elapsedTime);
                binormals[z * pointsX + x] = WaterSurfaceGenerator.
                            getBinormalAt(x*scale, z*scale, elapsedTime);
            }
        }
        
        this.setBuffer(Type.Position, 3, BufferUtils.createFloatBuffer(vertices));
        this.setBuffer(Type.Normal, 3, BufferUtils.createFloatBuffer(normals));
        this.setBuffer(Type.Tangent, 3, BufferUtils.createFloatBuffer(tangents));
        this.setBuffer(Type.Binormal, 3, BufferUtils.createFloatBuffer(binormals));
        this.updateBound();
    }
    
    public void updateGerstnerHeights(float elapsedTime, Geometry geom) {
        for(int z=0; z <pointsZ; z++) {
            for(int x=0; x <pointsX; x++) {
                vertices[z * pointsX + x] = geom.worldToLocal(
                        WaterSurfaceGenerator.getGerstnerPositionAt(x*scale, z*scale, elapsedTime)
                        .add(new Vector3f(x*scale, 0f, z*scale)), null);
                normals[z * pointsX + x] = WaterSurfaceGenerator.
                            getGerstnerNormalAt(x*scale, z*scale, elapsedTime);
                tangents[z * pointsX + x] = WaterSurfaceGenerator.
                            getGerstnerTangentAt(x*scale, z*scale, elapsedTime);
                binormals[z * pointsX + x] = WaterSurfaceGenerator.
                            getGerstnerBinormalAt(x*scale, z*scale, elapsedTime);
            }
        }
        
        this.setBuffer(Type.Position, 3, BufferUtils.createFloatBuffer(vertices));
        this.setBuffer(Type.Normal, 3, BufferUtils.createFloatBuffer(normals));
        this.setBuffer(Type.Tangent, 3, BufferUtils.createFloatBuffer(tangents));
        this.setBuffer(Type.Binormal, 3, BufferUtils.createFloatBuffer(binormals));
        this.updateBound();
    }
    
    public int getPointsX() {
        return pointsX;
    }
    
    public int getPointsZ() {
        return pointsZ;
    }
    
    public float getScale() {
        return scale;
    }
   
}
