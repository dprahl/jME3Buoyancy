package buoyancy.simple;

import buoyancy.surface.WaterSurfaceAppState;
import com.jme3.app.Application;
import com.jme3.app.SimpleApplication;
import com.jme3.app.state.AbstractAppState;
import com.jme3.app.state.AppStateManager;
import com.jme3.bullet.control.RigidBodyControl;
import com.jme3.input.InputManager;
import com.jme3.input.KeyInput;
import com.jme3.input.MouseInput;
import com.jme3.input.controls.ActionListener;
import com.jme3.input.controls.AnalogListener;
import com.jme3.input.controls.KeyTrigger;
import com.jme3.input.controls.MouseAxisTrigger;
import com.jme3.input.controls.MouseButtonTrigger;

/**
 *
 * @author User
 */
public class SimpleBoatInputAppState extends AbstractAppState {
    private SimpleApplication app;
    private InputManager inputManager;
    private SimpleBoatControl boatControl;
    private WaterSurfaceAppState waterState;
    
    @Override
    public void initialize(AppStateManager stateManager, Application app) {
        super.initialize(stateManager, app);
        //TODO: initialize your AppState, e.g. attach spatials to rootNode
        //this is called on the OpenGL thread after the AppState has been attached
        this.app = (SimpleApplication) app;
        this.inputManager = app.getInputManager();
        this.boatControl = stateManager.getState(SimpleBoatAppState.class).getBoatControl();
        this.waterState = stateManager.getState(WaterSurfaceAppState.class);
        setupKeyInputs();
    }
    
    @Override
    public void update(float tpf) {
        //TODO: implement behavior during runtime
    }
    
    @Override
    public void cleanup() {
        super.cleanup();
        //TODO: clean up what you initialized in the initialize method,
        //e.g. remove all spatials from rootNode
        //this is called on the OpenGL thread after the AppState has been detached
    }
    
    private void setupKeyInputs() {
        // Disable Default Mappings
        inputManager.deleteMapping(SimpleApplication.INPUT_MAPPING_MEMORY);
        
//        KeyTrigger myKeyTrigger = new KeyTrigger(KeyInput.KEY_SPACE);
//        MouseButtonTrigger myMouseButtonTrigger = new MouseButtonTrigger(MouseInput.BUTTON_LEFT);
//        MouseAxisTrigger myMouseAxisTrigger = new MouseAxisTrigger(MouseInput.AXIS_WHEEL, false);    

        // Create Key Triggers
        KeyTrigger pauseTrigger = new KeyTrigger(KeyInput.KEY_P);
        KeyTrigger debugMode = new KeyTrigger(KeyInput.KEY_B);
        KeyTrigger moveForward = new KeyTrigger(KeyInput.KEY_I);
        KeyTrigger turnLeft = new KeyTrigger(KeyInput.KEY_J);
        KeyTrigger moveBackward = new KeyTrigger(KeyInput.KEY_K);
        KeyTrigger turnRight = new KeyTrigger(KeyInput.KEY_L);
        KeyTrigger spaceBar = new KeyTrigger(KeyInput.KEY_SPACE);
        KeyTrigger resetTrigger = new KeyTrigger(KeyInput.KEY_R);
        MouseButtonTrigger leftMouseButton = new MouseButtonTrigger(MouseInput.BUTTON_LEFT);
        MouseButtonTrigger middleMouseButton = new MouseButtonTrigger(MouseInput.BUTTON_MIDDLE);
        MouseButtonTrigger rightMouseButton = new MouseButtonTrigger(MouseInput.BUTTON_RIGHT);
        MouseAxisTrigger mouseWheel = new MouseAxisTrigger(MouseInput.AXIS_WHEEL, true);
        
        // Add Mappings
        inputManager.addMapping("Pause Game", pauseTrigger);
        inputManager.addMapping("Debug Mode", debugMode);
        inputManager.addMapping("Reset", resetTrigger);
        inputManager.addMapping("Move Forward", moveForward);
        inputManager.addMapping("Turn Left", turnLeft);
        inputManager.addMapping("Move Backward", moveBackward);
        inputManager.addMapping("Turn Right", turnRight);
        inputManager.addMapping("Primary Action", leftMouseButton, spaceBar);
        inputManager.addMapping("Secondary Action", rightMouseButton);
        inputManager.addMapping("Tertiary Action", middleMouseButton);
        inputManager.addMapping("Mouse Wheel", mouseWheel);
        
        // Add Listeners for the Mappings
        inputManager.addListener(actionListener, new String[]{  "Pause Game", 
                                                                "Debug Mode", 
                                                                "Reset"});
        inputManager.addListener(actionListener, "Move Forward");
        inputManager.addListener(actionListener, "Turn Left");
        inputManager.addListener(actionListener, "Move Backward");
        inputManager.addListener(actionListener, "Turn Right");
//        inputManager.addListener(actionListener, "Primary Action");
//        inputManager.addListener(actionListener, "Secondary Action");
//        inputManager.addListener(actionListener, "Tertiary Action");
        inputManager.addListener(actionListener, new String[]{  "Primary Action",
                                                                "Secondary Action",
                                                                "Tertiary Action"});
        inputManager.addListener(analogListener, "Mouse Wheel");
        
//        // Test Multiple mappings per listener (array of Strings)
//        inputManager.addListener(analogListener, new String[]{"Left", "Right"});
//        // Test multiple listeners per mapping
//        inputManager.addListener(actionListener, "My Action");
//        inputManager.addListener(analogListener, "My Action");
    }
    
    private final ActionListener actionListener = new ActionListener() { 
        @Override
        public void onAction(String name, boolean keyPressed, float tpf) { 
//            if (name.equals("Pause Game") && !keyPressed) { // test?
//                isRunning = !isRunning;                    // action!
//            }
            if (name.equals("Pause Game") && !keyPressed) {
                waterState.setActive(!waterState.isActive());
                boatControl.setActive(!boatControl.isActive());
                
            }

            if (name.equals("Debug Mode") && !keyPressed) {
                // toggle debug mode
            }
            if(name.equals("Reset") && !keyPressed) {
                boatControl.debugReset();
            }

            // Moving Forward or Backward
            if(name.equals("Move Forward")) {
                boatControl.setMovingForward(keyPressed);
            }else if(name.equals("Move Backward")) {
                boatControl.setMovingBackward(keyPressed);
            }
            // Turning Left or Right
            if(name.equals("Turn Left")) {
                boatControl.setTurningLeft(keyPressed);
            }else if(name.equals("Turn Right")) {
                boatControl.setTurningRight(keyPressed);
            }
            
            System.out.println(name + " = " + keyPressed);            
        }
    };
    
    private final AnalogListener analogListener = new AnalogListener() { 
        @Override
        public void onAnalog(String name, float value, float tpf) { 
//            if (name.equals("Rotate")) {           // test?
//                player.rotate(0, value*speed, 0); // action!
//            }
            System.out.println(name + " = " + value);
        }
    };
    
}
