package buoyancy.simple;


import com.jme3.app.SimpleApplication;
import com.jme3.bullet.BulletAppState;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Vector3f;
import com.jme3.renderer.RenderManager;
import com.jme3.system.AppSettings;
import buoyancy.surface.WaterSurfaceAppState;
import buoyancy.debug.Utils;
import com.jme3.math.Quaternion;

/**
 * 
 * @author User
 */
public class SimpleBouyancyTest extends SimpleApplication {
    private BulletAppState bulletAppState;
    public static void main(String[] args) {
        AppSettings settings = new AppSettings(true);
        settings.setResolution(1024, 600);
        //settings.setAudioRenderer(null);
        settings.setTitle("Simple Bouyancy Test App");
        
        SimpleBouyancyTest app = new SimpleBouyancyTest();
        app.setSettings(settings);
        app.setShowSettings(false);
        app.start();
    }

    @Override
    public void simpleInitApp() {

        // use debug.Utils to setup XYZ widget at origin and some light
        rootNode.attachChild(Utils.orthoNormalBasis(assetManager));
        rootNode.addLight(Utils.sunLight(new Vector3f(-1f, -1f, 1f)));
        rootNode.addLight(Utils.sunLight(new Vector3f(0.25f, 0.25f, 0.25f), ColorRGBA.DarkGray));
        
        viewPort.setBackgroundColor(ColorRGBA.DarkGray);
        flyCam.setMoveSpeed(15f);
        flyCam.setDragToRotate(false);
        cam.lookAtDirection(Vector3f.UNIT_X, Vector3f.UNIT_Y);


        bulletAppState = new BulletAppState();
        stateManager.attach(bulletAppState);
        //bulletAppState.setDebugEnabled(true);
        
        /* add Water Surface */
        WaterSurfaceAppState fluidState = new WaterSurfaceAppState();
        stateManager.attach(fluidState);
        
        /* add Water Craft (after Water Surface) */
        SimpleBoatAppState boatState = new SimpleBoatAppState();
        stateManager.attach(boatState);
        
        /* add Input Controls */
        SimpleBoatInputAppState inputState = new SimpleBoatInputAppState();
        stateManager.attach(inputState);
        
    }

    @Override
    public void simpleUpdate(float tpf) {
        //TODO: add update code
    }

    @Override
    public void simpleRender(RenderManager rm) {
        //TODO: add render code
    }
    
    public BulletAppState getBulletAppState() {
        return bulletAppState;
    }
    
    
}
