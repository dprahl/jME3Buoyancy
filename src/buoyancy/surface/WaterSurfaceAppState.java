package buoyancy.surface;

import com.jme3.app.Application;
import com.jme3.app.SimpleApplication;
import com.jme3.app.state.AbstractAppState;
import com.jme3.app.state.AppStateManager;
import com.jme3.asset.AssetManager;
import com.jme3.asset.TextureKey;
import com.jme3.audio.AudioData.DataType;
import com.jme3.audio.AudioNode;
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.PhysicsTickListener;
import com.jme3.material.Material;
import com.jme3.material.RenderState;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Vector2f;
import com.jme3.math.Vector3f;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.texture.Texture;
import com.jme3.util.TangentBinormalGenerator;

/**
 *
 * @author User
 */
public class WaterSurfaceAppState extends AbstractAppState implements PhysicsTickListener {
    private SimpleApplication app;
    private AssetManager assetManager;
    private BulletAppState bulletAppState;
    private AudioNode audioNode;
    private Node rootNode;
    private boolean debug;
    private boolean debugNormals;
    private Mesh normalArrows;
    private Geometry normalArrowsGeom;
    
    private boolean useGerstnerWaves;
    
    private WaterSurfacePatch waterPatch;
    private Geometry waterPatchGeom;
    private Node waterPatchNode;
    
    private int linesX;
    private int linesZ;
    private float scale;
    
    private float elapsedBouyancyTime;
    private boolean isActive;
    
    @Override
    public void initialize(AppStateManager stateManager, Application app) {
        super.initialize(stateManager, app);
        this.app = (SimpleApplication) app;
        this.assetManager = this.app.getAssetManager();
        this.bulletAppState = app.getStateManager().getState(BulletAppState.class);
        this.rootNode = this.app.getRootNode();
        this.debug = true;
        this.debugNormals = false;
        this.elapsedBouyancyTime = 0f;
        this.linesX = 32; // num of waterPatch grid rows in x direction
        this.linesZ = 32; // num of waterPatch grid columns in z direction
        this.scale = 4f;  // scale of waterPatch cells in world units
        
        this.useGerstnerWaves = false;
        
        init();
    }
    
    private void init() {
          //System.out.println("Number of Wave Layers: "+ WaterSurfaceGenerator.getWaveLayers());
        
        // register this appState as a physics listener
        bulletAppState.getPhysicsSpace().addTickListener(this);
        
        // Mesh
        waterPatch = new WaterSurfacePatch( linesX, linesZ, scale);
        
        // Geometry
        waterPatchGeom = new Geometry("Water Patch Geometry", waterPatch);
        
        // Materials
        Material wireFrameMat = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
          wireFrameMat.setColor("Color", new ColorRGBA(0.75f, 0.75f, 1f, 1f)); //ColorRGBA.White);
          wireFrameMat.getAdditionalRenderState().setWireframe(true);
          wireFrameMat.getAdditionalRenderState().setFaceCullMode(RenderState.FaceCullMode.Off);
          
//        Material waterSurfaceMat = new Material(assetManager, "Common/MatDefs/Light/Lighting.j3md");
//          Texture waterSurfaceTexture = assetManager.loadTexture("Textures/WaterSurface/Water 0175b.jpg");
//          waterSurfaceTexture.setWrap(Texture.WrapMode.Repeat);
//          Texture waterSurfaceNormalMap = assetManager.loadTexture("Textures/WaterSurface/Water 0175bnormal.jpg");
//          waterSurfaceNormalMap.setWrap(Texture.WrapMode.Repeat);
//          waterSurfaceMat.setTexture("DiffuseMap", waterSurfaceTexture);
//          waterSurfaceMat.setTexture("NormalMap", waterSurfaceNormalMap);
//          waterSurfaceMat.setBoolean("UseMaterialColors",true);
//          waterSurfaceMat.setColor("Diffuse",new ColorRGBA(0.55f, 0.60f, 0.75f, 1f));
//          waterSurfaceMat.setColor("Ambient",ColorRGBA.White);
//          waterSurfaceMat.setColor("Specular",ColorRGBA.White);
//          waterSurfaceMat.setFloat("Shininess", 128f);  // [0,128]
//          waterSurfaceMat.getAdditionalRenderState().setFaceCullMode(RenderState.FaceCullMode.Off);
          
        Material waterSurfaceWhiteMat = new Material(assetManager, "Common/MatDefs/Light/Lighting.j3md");
          waterSurfaceWhiteMat.setBoolean("UseMaterialColors",true);
          waterSurfaceWhiteMat.setColor("Diffuse",new ColorRGBA(0.32f, 0.32f, 0.64f, 1f));
          waterSurfaceWhiteMat.setColor("Ambient",ColorRGBA.White); //DarkGray);
          waterSurfaceWhiteMat.setColor("Specular",ColorRGBA.White); //LightGray);
          waterSurfaceWhiteMat.setFloat("Shininess", 96f);  // [0,128]
          waterSurfaceWhiteMat.getAdditionalRenderState().setFaceCullMode(RenderState.FaceCullMode.Off);
          
        Material debugWaterSurfaceMat = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
          Texture debugTexture = assetManager.loadTexture("Interface/Logo/Monkey.jpg");
          debugTexture.setWrap(Texture.WrapMode.Repeat);
          debugWaterSurfaceMat.setTexture("ColorMap", debugTexture);
          //debugWaterSurfaceMat.setTexture("ColorMap", assetManager.loadTexture(new TextureKey("Interface/Logo/Monkey.jpg", true)));     
          debugWaterSurfaceMat.getAdditionalRenderState().setFaceCullMode(RenderState.FaceCullMode.Off);
          
        // showNormals Material is broken, only displays initial normals  
        Material showNormals = new Material(assetManager, "Common/MatDefs/Misc/ShowNormals.j3md");
        
        //Material proceduralTextureMat; // TODO
        
        waterPatchGeom.setMaterial(this.debug == true ? wireFrameMat : waterSurfaceWhiteMat);
        //waterPatchGeom.setMaterial( showNormals ); // not updated with mesh changes
        //waterPatchGeom.setMaterial(debugWaterSurfaceMat);
        
        waterPatch.scaleTextureCoordinates(new Vector2f(4f, 4f));
        //waterPatchGeom.setCullHint(Spatial.CullHint.Always); // to make water invisible
        
        // Node
        waterPatchNode = new Node("Water Patch Node");
        waterPatchNode.attachChild(waterPatchGeom);
        
        if(debugNormals) {
            //dTangentBinormalGenerator.generate(waterPatch);
            //normalArrows = TangentBinormalGenerator.genNormalLines(waterPatch, 0.25f);
            normalArrows = TangentBinormalGenerator.genTbnLines(waterPatch, 0.25f);
            
            normalArrowsGeom = new Geometry("Normal Arrows", normalArrows);
            normalArrowsGeom.setCullHint(Spatial.CullHint.Never);
            Material normalArrowMat = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
            normalArrowMat.setBoolean("VertexColor", true);
              //normalArrowMat.setColor("Color", ColorRGBA.White);
              //normalArrowMat.getAdditionalRenderState().setWireframe(true);
            normalArrowsGeom.setMaterial(normalArrowMat);
            waterPatchNode.attachChild(normalArrowsGeom);  
        }

          // to translate diagonally 1 cell
        //waterPatchNode.setLocalTranslation(linesX, 0f, linesZ);
          // to lift - BROKEN
        //waterPatchNode.setLocalTranslation(0f, 2f, 0f);
          // to center
        //waterPatchNode.setLocalTranslation((linesX-1) * -0.5f, 0f, (linesZ-1) * -0.5f);
        
        rootNode.attachChild(waterPatchNode);
        isActive = true;
//        initAudio();
    }
    
    public SimpleApplication getApp() {
        return this.app;
    }
    
    public Node getWaterPatchNode() {
        return waterPatchNode;
    }
    
    public Geometry getWaterPatchGeom() {
        return waterPatchGeom;
    }
    
    public WaterSurfacePatch getWaterPatch() {
        return waterPatch;
    }
    
    private void initAudio() {
        // ocean sound - keeps playing in a loop
        audioNode = new AudioNode(assetManager, "Sound/Environment/Ocean Waves.ogg", DataType.Stream);
        audioNode.setLooping(true);
        audioNode.setPositional(true);
        audioNode.setVolume(3);
        rootNode.attachChild(audioNode);
        audioNode.play();
    }
    
    @Override
    public void update(float tpf) {
        // move audio listener with cam
//        app.getListener().setLocation(app.getCamera().getLocation());
        
        if(debugNormals) {
            normalArrowsGeom.setMesh(
                    //TangentBinormalGenerator.genNormalLines(waterPatch, 0.25f));
                    TangentBinormalGenerator.genTbnLines(waterPatch, 0.25f));
        }
    }
    
    @Override
    public void cleanup() {
        super.cleanup();
        //bulletAppState.getPhysicsSpace().removeAll(waterPatchNode); // already gone?
        waterPatchNode.removeFromParent();
    }

    @Override
    public void prePhysicsTick(PhysicsSpace space, float tpf) {
        if(isActive){
            if(elapsedBouyancyTime > Float.MAX_VALUE - 1f) { 
                elapsedBouyancyTime -= Float.MAX_VALUE - 1f;
            }
            elapsedBouyancyTime += tpf;

            if(!useGerstnerWaves){
                waterPatch.updateHeights(elapsedBouyancyTime, waterPatchGeom);
            }else{
                waterPatch.updateGerstnerHeights(elapsedBouyancyTime, waterPatchGeom);
            }
        }
    }

    @Override
    public void physicsTick(PhysicsSpace space, float tpf) {
        // for polling state after prePhysicsTick has completed
    }
    
    public float getElapsedBouyancyTime() {
        return elapsedBouyancyTime;
    }
    
    public void resetElapsedBouyancyTime() {
        elapsedBouyancyTime = 0f;
    }
    
    public boolean isUsingGerstnerWaves() {
        return useGerstnerWaves;
    }
    
    public boolean isActive() {
        return isActive;
    }
    
    public void setActive(boolean isActive) {
        this.isActive = isActive;
    }
    
}
