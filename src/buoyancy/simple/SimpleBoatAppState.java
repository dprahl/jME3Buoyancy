package bouyancy.simple;

import com.jme3.app.Application;
import com.jme3.app.SimpleApplication;
import com.jme3.app.state.AbstractAppState;
import com.jme3.app.state.AppStateManager;
import com.jme3.asset.AssetManager;
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.collision.shapes.HullCollisionShape;
import com.jme3.bullet.control.RigidBodyControl;
import com.jme3.material.Material;
import com.jme3.material.RenderState;
import com.jme3.math.ColorRGBA;
import com.jme3.math.FastMath;
import com.jme3.math.Vector3f;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.shape.Box;
import com.jme3.scene.shape.Sphere;
import com.jme3.texture.Texture;
import bouyancy.surface.WaterSurfaceAppState;
import bouyancy.debug.*;
import com.jme3.light.AmbientLight;
import com.jme3.math.Quaternion;
import com.jme3.scene.VertexBuffer;
import com.jme3.util.BufferUtils;



/**
 *
 * @author User
 */
public class SimpleBoatAppState extends AbstractAppState { 
    private SimpleApplication app;
    private AssetManager assetManager;
    private BulletAppState bulletAppState;
    private WaterSurfaceAppState waterSurfaceAppState;
    private Node rootNode;
    private Node boatNode;
    private RigidBodyControl rigidBodyControl;
    private SimpleBoatControl boatControl;
    private Quaternion initialRotation;
    private Vector3f initialPosition;
    private boolean debug;
    
    @Override
    public void initialize(AppStateManager stateManager, Application app) {
        super.initialize(stateManager, app);
        this.app = (SimpleApplication) app;
        this.assetManager = this.app.getAssetManager();
        this.bulletAppState = app.getStateManager().getState(BulletAppState.class);
        this.waterSurfaceAppState = app.getStateManager().getState(WaterSurfaceAppState.class);
        this.rootNode = this.app.getRootNode();
        this.debug = true;
        
        boatNode = simpleBoat();
        if(debug) { 
            boatNode.attachChild(Utils.orthoNormalBasis(assetManager));
        }
        rootNode.attachChild(boatNode);
    }
    
    @Override
    public void update(float tpf) {
        super.update(tpf);
    }
    
    @Override
    public void cleanup() {
        super.cleanup();
        //bulletAppState.getPhysicsSpace().removeAll(boatNode); // already gone?
        boatNode.removeFromParent();
    }
    
    public SimpleBoatControl getBoatControl() {
        return boatControl;
    }
    
    public RigidBodyControl getRigidBodyControl() {
        return rigidBodyControl;
    }
    
    private Mesh extractMesh(Spatial subject) { // Limited Functionality
        /* returns first Mesh found from first Geometry found in given Spatial */
        if(subject instanceof Geometry) {
            return ((Geometry)subject).getMesh();
        }
        else if (subject instanceof Node) {
            return extractMesh(((Node) subject).getChild(0));
        }
        return null;
    }
    
    private Node simpleBoat() {
        
        // creates boat node 
        // creates boat geom and attaches to boat node
        // creates boat mesh and attaches to boat geom
        // creates boat material and attaches to boat geom
        // creates boat rigid body control and attaches to boat node
        // creates boat collision shape and attaches to boat rigid body control
        // creates boat water craft control and attaches to boat node
        // returns node with Mesh, Geom, Material, RigidBodyControl, and WaterCraftControl
        

        // Load a model and extract the mesh 
//        Spatial boatModel = assetManager.loadModel("Models/Boat/boat.j3o");
//        Mesh boatMesh = extractMesh(boatModel); /* could potentially be null */

        SimpleBoatMesh boatMesh = new SimpleBoatMesh();

        //Box boatMesh = new Box(new Vector3f(-1.5f,-1.5f,-1.5f), new Vector3f(1.5f,1.5f,1.5f));

        //Sphere boatMesh = new Sphere(8, 12, 1.5f);
        
        
        // calculate offset of mesh origin to center of Geometry
        Vector3f[] principalVertices = BufferUtils.getVector3Array(boatMesh.getFloatBuffer(VertexBuffer.Type.Position));
        Vector3f centerOfGravityOffset = Utils.calculateCenterOfMass(boatMesh);
        for(Vector3f vertex : principalVertices) {
            vertex.subtractLocal(centerOfGravityOffset);
        }
        boatMesh.setBuffer(VertexBuffer.Type.Position, 3, BufferUtils.createFloatBuffer(principalVertices));
        boatMesh.updateBound();

        // create Node and Geometry from boatMesh
        Geometry boatGeom = new Geometry("Boat Geometry", boatMesh);
        boatNode = new Node("Boat Node");
        //float mass = Utils.calculateMass(boatMesh); // 1kg = 2.205lbs
        float mass = 200f;
        System.out.println("Mass: "+ mass);

        // Materials - mostly debug
        Material texturedBoatMat = new Material(assetManager, "Common/MatDefs/Light/Lighting.j3md");
          Texture testBoatTexture = assetManager.loadTexture("Models/Boat/boat.png");
          Texture testBoatNormalMap = assetManager.loadTexture("Models/Boat/boat_normal.png");
          texturedBoatMat.setTexture("DiffuseMap", testBoatTexture);
          texturedBoatMat.setTexture("NormalMap", testBoatNormalMap);
          texturedBoatMat.setBoolean("UseMaterialColors",true);
          texturedBoatMat.setColor("Diffuse",new ColorRGBA(0.55f, 0.60f, 0.75f, 1f));
          texturedBoatMat.setColor("Ambient",ColorRGBA.White);
          texturedBoatMat.setColor("Specular",ColorRGBA.White);
          texturedBoatMat.setFloat("Shininess", 32f);  // [0,128]

        Material wireFrameMat = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
          wireFrameMat.setColor("Color", ColorRGBA.Green);
          wireFrameMat.getAdditionalRenderState().setWireframe(true);
          wireFrameMat.getAdditionalRenderState().setLineWidth(2f);
          //wireFrameMat.getAdditionalRenderState().setFaceCullMode(RenderState.FaceCullMode.Off); // to disable backface culling

        Material boatMat = new Material(assetManager, "Common/MatDefs/Light/Lighting.j3md");
          Texture boatTexture = assetManager.loadTexture("Textures/wood-textures-high-quality-1.jpg");
          //Texture boatNormalMap = assetManager.loadTexture("Textures/WaterSurface/Water 0175bnormal.jpg");
          boatMat.setTexture("DiffuseMap", boatTexture);
          //boatMat.setTexture("NormalMap", boatNormalMap); // no tangents
          boatTexture.setWrap(Texture.WrapMode.Repeat);

        Material whiteMat = new Material(assetManager, "Common/MatDefs/Light/Lighting.j3md");
          whiteMat.setBoolean("UseMaterialColors",true);
          whiteMat.setColor("Diffuse", ColorRGBA.Brown);
          whiteMat.setColor("Ambient",ColorRGBA.LightGray);
          whiteMat.setColor("Specular",ColorRGBA.LightGray);
          whiteMat.setFloat("Shininess", 8f);  /* [0,128] */

        Material showNormals = new Material(assetManager, "Common/MatDefs/Misc/ShowNormals.j3md");

        boatGeom.setMaterial(wireFrameMat);
        //boatGeom.setMaterial(showNormals); boatGeom.scale(0.9999f); // to counter Z fighting
        //boatGeom.setMaterial(whiteMat); boatGeom.scale(0.9999f);  boatGeom.addLight(new AmbientLight(ColorRGBA.LightGray)); // boatGeom.scale(0.9999f);
        //boatGeom.setMaterial(boatMat);
        //boatGeom.setMaterial(texturedBoatMat);
        //boatGeom.setMaterial(debug ? wireFrameMat : boatMat);
        //boatGeom.setMaterial(debug ? showNormals : whiteMat);

        //boatGeom.setCullHint(Spatial.CullHint.Always); // to make invisible

        boatNode.attachChild(boatGeom);

        // Create physics representation with RigidBodyControl AND SimpleBoatControl
        HullCollisionShape convexHull = new HullCollisionShape(boatMesh);
        rigidBodyControl = new RigidBodyControl(convexHull, mass); 
        bulletAppState.getPhysicsSpace().add(rigidBodyControl);
        boatNode.addControl(rigidBodyControl);

        boatControl = new SimpleBoatControl(waterSurfaceAppState, rigidBodyControl, boatGeom);
        boatNode.addControl(boatControl);

        // debug starting rotation
        initialRotation = rigidBodyControl.getPhysicsRotation();
//        initialRotation = new Quaternion().fromAngleAxis(FastMath.PI, Vector3f.UNIT_Y);
//        initialRotation = new Quaternion().fromAngleAxis(FastMath.QUARTER_PI, Vector3f.UNIT_Z);
        
        // debug starting translation
        initialPosition = new Vector3f(12f, 2.75f, 10f);
        
        initializeOrientation();
        
        return boatNode;
    }
    
    public void initializeOrientation() {
        rigidBodyControl.setPhysicsLocation(initialPosition);
        rigidBodyControl.setPhysicsRotation(initialRotation);
    }
    
}
