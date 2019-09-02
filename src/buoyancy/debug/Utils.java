package bouyancy.debug;

import com.jme3.asset.AssetManager;
import com.jme3.light.DirectionalLight;
import com.jme3.material.Material;
import com.jme3.math.ColorRGBA;
import com.jme3.math.FastMath;
import com.jme3.math.Vector2f;
import com.jme3.math.Vector3f;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.scene.VertexBuffer;
import com.jme3.scene.debug.Arrow;
import com.jme3.scene.mesh.IndexBuffer;
import com.jme3.util.BufferUtils;
import bouyancy.surface.WaveProperties;
import java.nio.FloatBuffer;
import java.nio.IntBuffer;
import java.nio.ShortBuffer;

/**
 *
 * @author User
 */
public class Utils {
    public static Node orthoNormalBasis(AssetManager assetManager) {
        Node widget = new Node("OrthoNormal Basis");
        
        Material redMat = new Material(assetManager,"Common/MatDefs/Misc/Unshaded.j3md");
          redMat.setColor("Color", ColorRGBA.Red);
          redMat.getAdditionalRenderState().setLineWidth(2f);
        Material greenMat = new Material(assetManager,"Common/MatDefs/Misc/Unshaded.j3md");
          greenMat.setColor("Color", ColorRGBA.Green);
          greenMat.getAdditionalRenderState().setLineWidth(2f);
        Material blueMat = new Material(assetManager,"Common/MatDefs/Misc/Unshaded.j3md");
          blueMat.setColor("Color", ColorRGBA.Blue);
          blueMat.getAdditionalRenderState().setLineWidth(2f);
        
        Arrow redArrow = new Arrow(Vector3f.UNIT_X.clone());
          Geometry xArrow  = new Geometry("Red Arrow", redArrow);
        Arrow greenArrow   = new Arrow(Vector3f.UNIT_Y.clone());
          Geometry yArrow  = new Geometry("Green Arrow",   greenArrow);
        Arrow blueArrow  = new Arrow(Vector3f.UNIT_Z.clone());
          Geometry zArrow  = new Geometry("Blue Arrow",   blueArrow);
        
        xArrow.setMaterial(redMat);
        yArrow.setMaterial(greenMat);
        zArrow.setMaterial(blueMat);
        
        widget.attachChild(xArrow);
        widget.attachChild(yArrow);
        widget.attachChild(zArrow);
        
        return widget;
    }
    
    public static DirectionalLight sunLight(Vector3f direction) {
        return sunLight(direction, ColorRGBA.White);
    }
    
    public static DirectionalLight sunLight(Vector3f direction, ColorRGBA color) {
        DirectionalLight sun = new DirectionalLight();
        sun.setDirection(new Vector3f(direction).normalizeLocal());
        sun.setColor(color);
        return sun;
    }
    
    public static WaveProperties getRandomWave() {
        float wavelength = 10 * (1f + FastMath.nextRandomFloat());
        float amplitude = 0.1f * FastMath.nextRandomFloat();
        float speed = 1.5f + FastMath.nextRandomFloat();
        Vector2f direction = randomDirection2d();
        return new WaveProperties(wavelength, amplitude, speed, direction);
    }
    
//    public static WaveProperties getRandomCircularWave() {
//        float wavelength = 10 * (1f + FastMath.nextRandomFloat());
//        float amplitude = 0.1f * FastMath.nextRandomFloat();
//        float speed = 1.5f + FastMath.nextRandomFloat();
//        int range = 10;
//        Vector2f center = new Vector2f(
//                FastMath.nextRandomInt(-range, range), 
//                FastMath.nextRandomInt(-range, range));
//        return new WaveProperties(wavelength, amplitude, speed, center, true);
//    }
    
    public static Vector2f randomDirection2d() {
        float x = FastMath.nextRandomFloat() * 2 - 1;
        float y = FastMath.nextRandomFloat() * 2 - 1;
        return new Vector2f(x, y).normalizeLocal();
    }
    
    public static WaveProperties getRandomWaveConstrained(Vector2f windDir) {
        float wavelength = 10f + FastMath.nextRandomFloat();
        float amplitude = 0.1f * FastMath.nextRandomFloat();
        float speed = 1.5f + FastMath.nextRandomFloat();
        Vector2f direction = randomDirectionConstrained(windDir);
        return new WaveProperties(wavelength, amplitude, speed, direction);
    }
    
    public static Vector2f randomDirectionConstrained(Vector2f medianDir) {
        float x = medianDir.getX();
        float y = medianDir.getY();
        x += FastMath.nextRandomInt(-1000, 1000)* 0.001f;
        y += FastMath.nextRandomInt(-1000, 1000)* 0.001f;
        return new Vector2f(x, y);
    }
    
    public static Vector3f clampVelocity(Vector3f velocity, float maxSpeed) {
        float currentSpeed = velocity.length();
        if(currentSpeed > maxSpeed) {
           velocity.multLocal(maxSpeed/currentSpeed);
        }
        return velocity;
    }
    
    public static Vector3f capVelocity(Vector3f velocity, float maxSpeed) {
        float speed = velocity.length();
        if(speed > maxSpeed) {
            return velocity.mult(maxSpeed/speed);
        }else{
            return velocity;
        }
    }
    
    public static float mapToRange( float min, float max, 
                                    float newMin, float newMax, 
                                    float value) {
        float result = newMin + ((newMax - newMin) / (max - min)) * (value - min);
//        if(Float.isNaN(result)){
//            return newMin;
//        }
        return result;
    }
    
    @Deprecated
    public static float travelMap(float distanceFactor) {
        return -1f + 3f * distanceFactor;
    }
    
    /*
    @Deprecated
    public Vector3f getCenterOfMass(Mesh mesh) {
        FloatBuffer verticesBuff = mesh.getFloatBuffer(VertexBuffer.Type.Position);
        Vector3f[] vertices = BufferUtils.getVector3Array(verticesBuff);
        IndexBuffer indicesBuff = mesh.getIndexBuffer();
        int[] indices = new int[indicesBuff.size()];
        for(int i = 0; i < indices.length; i++) {
            indices[i] = indicesBuff.get(i);
        }
        Vector3f center = new Vector3f(0,0,0);
        for(int i = 0; i < indices.length; i +=3) {
            Vector3f simplexCenter =    vertices[indices[i]].
                                        add(vertices[indices[i+1]]).
                                        add(vertices[indices[i+2]]).
                                        divide(4f);
            center.addLocal(simplexCenter);
        }
        center.divideLocal(indices.length / 3);
        return center;
    }
    
    @Deprecated
    public Vector3f getCenterOfMassOffset(Mesh mesh) {
        FloatBuffer verticesBuff = mesh.getFloatBuffer(VertexBuffer.Type.Position);
        Vector3f[] vertices = BufferUtils.getVector3Array(verticesBuff);
        IndexBuffer indicesBuff = mesh.getIndexBuffer();
        Vector3f center = new Vector3f(0f,0f,0f);
        int triangleCount = 0;
        for(int i = 0; i < indicesBuff.size(); i +=3) {
            Vector3f simplexCenter =    vertices[indicesBuff.get(i)].
                                        add(vertices[indicesBuff.get(i+1)]).
                                        add(vertices[indicesBuff.get(i+2)]).
                                        divide(4f);
            center.addLocal(simplexCenter);
            triangleCount++;
        }
        center.divideLocal(triangleCount);
        return center;
    }
    */
    
    public static Vector3f calculateCenterOfMass(Mesh mesh) {
        FloatBuffer vertBuff = mesh.getFloatBuffer(VertexBuffer.Type.Position);
        Vector3f[] verts = BufferUtils.getVector3Array(vertBuff);
        IndexBuffer indices = mesh.getIndexBuffer();
        Face[] faces = new Face[mesh.getTriangleCount()];
        int faceCount = 0;
        for(int i = 0; i < indices.size(); i += 3) {
            faces[faceCount] = new Face(verts[indices.get(i)], verts[indices.get(i+1)], verts[indices.get(i+2)]);
            faceCount++;
        }
        float mass = 0;
        Vector3f triangleCenter;
        float faceMass;
        Vector3f massCenter = new Vector3f(0, 0, 0);
        for (Face face : faces) {
            triangleCenter = face.getCenter();
            faceMass = face.getSurface();
            mass += faceMass;
            massCenter.x += faceMass * triangleCenter.x;
            massCenter.y += faceMass * triangleCenter.y;
            massCenter.z += faceMass * triangleCenter.z;
        }
        massCenter.x /= mass;
        massCenter.y /= mass;
        massCenter.z /= mass;
        return massCenter;
    }
    
    public static float calculateMass(Mesh mesh) {
        FloatBuffer vertBuff = mesh.getFloatBuffer(VertexBuffer.Type.Position);
        Vector3f[] verts = BufferUtils.getVector3Array(vertBuff);
        IndexBuffer indices = mesh.getIndexBuffer();
        Face[] faces = new Face[mesh.getTriangleCount()];
        int faceCount = 0;
        for(int i = 0; i < indices.size(); i += 3) {
            faces[faceCount] = new Face(verts[indices.get(i)], verts[indices.get(i+1)], verts[indices.get(i+2)]);
            faceCount++;
        }
        float mass = 0;
        for (Face face : faces) {
            float faceMass = face.getSurface();
            mass += faceMass;
        }
        return mass * 2;
    }
    
     @Deprecated
    public static Vector3f getLocalCenterOfMass(Mesh mesh) {
        FloatBuffer vertBuff = mesh.getFloatBuffer(VertexBuffer.Type.Position);
        Vector3f[] vertices = BufferUtils.getVector3Array(vertBuff);
        IndexBuffer indices = mesh.getIndexBuffer();
        
        Vector3f center = new Vector3f();
        float volume = 0f;
        for(int i=0; i<indices.size(); i+=3) {
                Vector3f a = vertices[indices.get(i)];
                Vector3f b = vertices[indices.get(i+1)];
                Vector3f c = vertices[indices.get(i+2)];
                float v = a.cross(b).dot(c) / 6f; // + or -
                volume += v;
                Vector3f geoCenter = a.add(b.add(c)).divide(4f).mult(v);
                center.addLocal(geoCenter);
        }
        return center.divide(volume);
    }
    
    
    public static class Face {
        private Vector3f p1;
        private Vector3f p2;
        private Vector3f p3;

        public Face(Vector3f p1, Vector3f p2, Vector3f p3) {
            this.p1 = p1;
            this.p2 = p2;
            this.p3 = p3;
        }
        
        public Vector3f getP1() {
            return p1;
        }
        
        public void setP1(Vector3f p1) {
            this.p1 = p1;
        }
        
        public Vector3f getP2() {
            return p2;
        }
        
        public void setP2(Vector3f p2) {
            this.p2 = p2;
        }
        
        public Vector3f getP3() {
            return p3;
        }
        
        public void setP3(Vector3f p3) {
            this.p3 = p3;
        }

        public Vector3f getCenter() {
            return p1.add(p2).add(p3).divide(3f);
        }

        public float getSurface() {
            float x1 = p1.x - p2.x;
            float x2 = p1.y - p2.y;
            float x3 = p1.z - p2.z;
            float y1 = p1.x - p3.x;
            float y2 = p1.y - p3.y;
            float y3 = p1.z - p3.z;
            return  0.5f * FastMath.sqrt(
                                    FastMath.sqr(x2 * y3 - x3 * y2) + 
                                    FastMath.sqr(x3 * y1 - x1 * y3) + 
                                    FastMath.sqr(x1 * y2 - x2 * y1) );
        }
    }// End of Face
        
}
