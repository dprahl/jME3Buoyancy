package bouyancy.debug;

import com.jme3.math.Triangle;
import com.jme3.math.Vector2f;
import com.jme3.math.Vector3f;
import com.jme3.scene.Mesh;
import com.jme3.scene.VertexBuffer;
import com.jme3.util.BufferUtils;

/**
 *
 * @author User
 */
public class SimpleBoatMesh extends Mesh {
        Vector3f[] vertices;
        Vector3f[] normals;
        Vector2f[] texCoords;
        int[] indices;
        private boolean isSmoothShaded;
        
    public SimpleBoatMesh() {
        isSmoothShaded = false;
        init();
    }
    
    public SimpleBoatMesh(boolean smoothShaded) {
        isSmoothShaded = smoothShaded;
        init();
    }
        
    private void init() {
        vertices = new Vector3f[] {
              //top center
            new Vector3f( 0.0f, 0.0f, 5.0f),// 0
              //top left
            new Vector3f( 2.0f, 0.0f, 2.0f),// 1
            new Vector3f( 2.0f, 0.0f,-1.0f),// 2
            new Vector3f( 1.0f, 0.0f,-5.0f),// 3
              //top right
            new Vector3f(-1.0f, 0.0f,-5.0f),// 4
            new Vector3f(-2.0f, 0.0f,-1.0f),// 5
            new Vector3f(-2.0f, 0.0f, 2.0f),// 6
              //middle center
            new Vector3f( 0.0f,-1.0f, 4.0f),// 7
              //middle left
            new Vector3f( 1.5f,-1.0f, 2.0f),// 8
            new Vector3f( 1.5f,-1.0f,-1.0f),// 9
            new Vector3f( 1.0f,-1.0f,-5.0f),//10
              //middle right
            new Vector3f(-1.0f,-1.0f,-5.0f),//11
            new Vector3f(-1.5f,-1.0f,-1.0f),//12
            new Vector3f(-1.5f,-1.0f, 2.0f),//13
              //bottom center row
            new Vector3f( 0.0f,-2.0f, 2.0f),//14
            new Vector3f( 0.0f,-2.0f,-1.0f),//15
            new Vector3f( 0.0f,-2.0f,-5.0f)//16
                  //top
                , new Vector3f( 0.0f, 0.0f, 5.0f),//17
                new Vector3f( 2.0f, 0.0f, 2.0f),// 18
                new Vector3f( 2.0f, 0.0f,-1.0f),// 19
                new Vector3f( 1.0f, 0.0f,-5.0f),// 20
                new Vector3f(-1.0f, 0.0f,-5.0f),// 21
                new Vector3f(-2.0f, 0.0f,-1.0f),// 22
                new Vector3f(-2.0f, 0.0f, 2.0f),// 23
        };
        
          // 30 triangles (25 without top)
        indices = new int[] { // new int[90]
              //left
            0,7,1, 1,7,8, 1,8,2, 2,8,9, 2,9,10, 2,10,3, 
            7,14,8, 8,14,9, 9,14,15, 9,15,10, 10,15,16, 
              //right
            4,11,5, 5,11,12, 5,12,13, 5,13,6, 6,13,7, 6,7,0, 
            11,16,15, 11,15,12, 12,15,14, 12,14,13, 13,14,7,
              //back
            3,10,4, 4,10,11, 
            10,16,11
              //top
              , 17,18,23, 18,19,23, 23,19,22, 19,20,22, 22,20,21
        };
        
        /*    
          //set normals old fashioned way, 
          //cross product of 2 triangle edges
          //for each vertex sum the normals of all triangles sharing
          // that vertex and normalize total result
        normals = new Vector3f[vertices.length];
          //init all to zero vector rather than null
        for(int i =0; i <normals.length; i++){
            normals[i] = new Vector3f(0f,0f,0f);
        }
        Vector3f tempVecA = new Vector3f();
        Vector3f tempVecB = new Vector3f();
        Vector3f tempVecX = new Vector3f();
          //for every 3 entries into indices (1 triangle at tempVecA time)
          //get normal of triangle, 
          //add to normal (sum) of each vertex referenced by the 3 indices entries
        for(int i =0; i <=indices.length-3; i+=3) {
              //get first edge vector from 2 triangle points
            tempVecA = vertices[indices[i]].subtract(vertices[indices[i+1]]).normalize();
              //get second edge vector from 2 triangle points
            tempVecB = vertices[indices[i]].subtract(vertices[indices[i+2]]).normalize();
              //get cross product of 2 edge vectors
            tempVecX = tempVecA.cross(tempVecB).normalize();
              //add this normal on top of the existing normal sum
              //of each of the 3 vertices that makes up the triangle
            normals[indices[i]].addLocal(tempVecX);
            normals[indices[i+1]].addLocal(tempVecX);
            normals[indices[i+2]].addLocal(tempVecX);
        }
          //after summing, normalize all vertex normals
        for(Vector3f normal : normals){
            normal.normalizeLocal();
        }
        */
        
          //normals hardcoded for speed
        normals = new Vector3f[] {
            new Vector3f(-4.344398E-8f, -0.7071068f, 0.7071068f),
            new Vector3f(0.81700885f, -0.45040065f, 0.36004993f),
            new Vector3f(0.93490326f, -0.3431345f, -0.09063482f),
            new Vector3f(0.61541224f, 0.0f, -0.78820544f),
            new Vector3f(-0.39704818f, 0.0f, -0.91779774f),
            new Vector3f(-0.93490326f, -0.3431345f, -0.09063482f),
            new Vector3f(-0.81700885f, -0.4504007f, 0.36004993f),
            new Vector3f(1.378521E-8f, -0.7515052f, 0.6597272f),
            new Vector3f(0.76633924f, -0.6103674f, 0.20043887f),
            new Vector3f(0.71282107f, -0.70035386f, -0.03729133f),
            new Vector3f(0.6192894f, -0.39339167f, -0.67950255f),
            new Vector3f(-0.70592034f, -0.4484223f, -0.54826456f),
            new Vector3f(-0.71282107f, -0.70035386f, -0.03729133f),
            new Vector3f(-0.76633924f, -0.6103674f, 0.20043886f),
            new Vector3f(2.420528E-8f, -0.9877592f, 0.15598625f),
            new Vector3f(1.2573631E-8f, -0.99957407f, -0.029183505f),
            new Vector3f(0.0f, -0.81649655f, -0.5773502f)
              //top
              , new Vector3f(0.0f, 1.0f, 0.0f),
              new Vector3f(0.0f, 1.0f, 0.0f),
              new Vector3f(0.0f, 1.0f, 0.0f),
              new Vector3f(0.0f, 1.0f, 0.0f),
              new Vector3f(0.0f, 1.0f, 0.0f),
              new Vector3f(0.0f, 1.0f, 0.0f),
              new Vector3f(0.0f, 1.0f, 0.0f)
        };
        
          //texCoords poorly hardcoded
        texCoords = new Vector2f[] {
            new Vector2f(1.0f, 1.0f),// 0
            new Vector2f(0.7f, 1.0f),// 1
            new Vector2f(0.4f, 1.0f),// 2
            new Vector2f(0.0f, 1.0f),// 3
            new Vector2f(0.0f, 1.0f),// 4
            new Vector2f(0.4f, 1.0f),// 5
            new Vector2f(0.7f, 1.0f),// 6
            new Vector2f(0.9f, 0.5f),// 7
            new Vector2f(0.7f, 0.5f),// 8
            new Vector2f(0.4f, 0.5f),// 9
            new Vector2f(0.0f, 0.5f),//10
            new Vector2f(0.0f, 0.5f),//11
            new Vector2f(0.4f, 0.5f),//12
            new Vector2f(0.7f, 0.5f),//13
            new Vector2f(0.7f, 0.0f),//14
            new Vector2f(0.4f, 0.0f),//15
            new Vector2f(0.0f, 0.0f) //16
              //top
              , new Vector2f(1.0f, 4.0f),//17
              new Vector2f(2.0f, 2.8f),//18
              new Vector2f(2.0f, 1.6f),//19
              new Vector2f(1.5f, 0.0f),//20
              new Vector2f(0.5f, 0.0f),//21
              new Vector2f(0.0f, 1.6f),//22
              new Vector2f(0.0f, 2.8f)//23
        };
        
        if(!isSmoothShaded) {
            unshareVertices();
        }
        
        this.setBuffer(VertexBuffer.Type.Position, 3, BufferUtils.createFloatBuffer(vertices));
        this.setBuffer(VertexBuffer.Type.Normal, 3, BufferUtils.createFloatBuffer(normals));
        this.setBuffer(VertexBuffer.Type.TexCoord, 2, BufferUtils.createFloatBuffer(texCoords));
        this.setBuffer(VertexBuffer.Type.Index, 1, BufferUtils.createIntBuffer(indices));
        this.updateBound();
    }
    
    private void unshareVertices() {
        int length = indices.length;
        Vector3f[] newVertices = new Vector3f[length];
        Vector3f[] newNormals = new Vector3f[length];
        Vector2f[] newTexCoords = new Vector2f[length];
        int[] newIndices = new int[length];
        
        Vector3f pointa;
        Vector3f pointb;
        Vector3f pointc;
        Vector3f normal;
        
        for(int i =0; i < length; i+=3) {
            
            // for each triangle, get the vertex found at each index
            pointa = vertices[indices[i]];
            pointb = vertices[indices[i+1]];
            pointc = vertices[indices[i+2]];
            
            // vertices
            newVertices[i] = pointa.clone();
            newVertices[i+1] = pointb.clone();
            newVertices[i+2] = pointc.clone();
            
            // normals - all three vertices of triangle share the same face normal
            normal = pointb.clone();
            normal.subtractLocal(pointa).crossLocal(
                    pointc.x - pointa.x, pointc.y - pointa.y, pointc.z - pointa.z);
            normal.normalizeLocal();
            newNormals[i] = normal.clone();
            newNormals[i+1] = normal.clone();
            newNormals[i+2] = normal.clone();
            
            // texCoords - copy the same texCoords as the vertex that was cloned
            newTexCoords[i] = texCoords[indices[i]];
            newTexCoords[i+1] = texCoords[indices[i+1]];
            newTexCoords[i+2] = texCoords[indices[i+2]];
            
            // each triangle vertex will be indexed in the order it was cloned
            newIndices[i] = i;
            newIndices[i+1] = i+1;
            newIndices[i+2] = i+2;
        }
        
        vertices = newVertices;
        normals = newNormals;
        texCoords = newTexCoords; 
        indices = newIndices;
    }
    
    
}
