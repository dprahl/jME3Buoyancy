package buoyancy.simple;

import buoyancy.debug.Utils;
import buoyancy.surface.WaterSurfacePatch;
import buoyancy.surface.WaterSurfaceAppState;
import buoyancy.surface.WaterSurfaceGenerator;
import com.jme3.app.SimpleApplication;
import com.jme3.bounding.BoundingBox;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.PhysicsTickListener;
import com.jme3.bullet.control.RigidBodyControl;
import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.input.ChaseCamera;
import com.jme3.material.Material;
import com.jme3.material.RenderState.FaceCullMode;
import com.jme3.math.ColorRGBA;
import com.jme3.math.FastMath;
import com.jme3.math.Quaternion;
import com.jme3.math.Triangle;
import com.jme3.math.Vector2f;
import com.jme3.math.Vector3f;
import com.jme3.renderer.RenderManager;
import com.jme3.renderer.ViewPort;
import com.jme3.scene.CameraNode;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.VertexBuffer;
import com.jme3.scene.control.AbstractControl;
import com.jme3.scene.control.CameraControl.ControlDirection;
import com.jme3.scene.control.Control;
import com.jme3.scene.mesh.IndexBuffer;
import com.jme3.scene.plugins.OBJLoader;
import com.jme3.scene.shape.Box;
import com.jme3.util.BufferUtils;
import com.jme3.util.TangentBinormalGenerator;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

/**
 *
 * @author User
 */
public class SimpleBoatControl extends AbstractControl implements PhysicsTickListener {
    private final WaterSurfaceAppState waterSurfaceAppState;
    private final RigidBodyControl rigidBodyControl;
    private final Node principalNode;
    private final Geometry principalGeom;
    private final Mesh principalMesh;
    private Geometry submergedGeom;
    private Mesh submergedMesh;
    private Geometry waterLineGeom;
    private Mesh waterLineMesh;
    private Geometry splitSubGeom;
    private Mesh splitSubMesh;
    private Geometry splitSubRejectedGeom;
    private Mesh splitSubRejectedMesh;
    private Geometry splitSubNormalsGeom;
    private Mesh splitSubNormalsMesh;
    private Geometry forcePointGeom;
    private Mesh forcePointMesh;
    private Geometry forceVectorGeom;
    private Mesh forceVectorMesh;
    private Geometry submergedTriangleVelocityGeom;
    private Mesh submergedTriangleVelocityMesh;
    private Geometry principalTriangleVelocityGeom;
    private Mesh principalTriangleVelocityMesh;
    private Geometry resistanceForceVectorGeom;
    private Mesh resistanceForceVectorMesh;
    private Geometry slammingForceVectorGeom;
    private Mesh slammingForceVectorMesh;
    private int arrayIndex;
    private final Vector3f[] principalVertices;
    private final int[] principalIndices;
    private final int numTris;
        private final float[] principalAreas;
        private float[] submergedPrincipalAreas;
        private float[] submergedPrincipalAreasPrevious;
        private Vector3f[] triangleVelocities;
        private Vector3f[] triangleVelocitiesPrevious;        
    private final ArrayList<Vector3f> waterLineVerts;
    private final ArrayList<Vector3f> submergedVertices;
    private final ArrayList<Integer> submergedIndices;
    private final ArrayList<Vector3f> splitSubVertices;
    private final ArrayList<Integer> splitSubIndices;
    private final ArrayList<Vector3f> splitSubRejectedVertices;
    private final ArrayList<Integer> splitSubRejectedIndices;
    private final ArrayList<Vector3f> splitSubNormalsVertices;
    private final ArrayList<Vector3f> bouyancyForcePointVertices;
    private final ArrayList<Vector3f> bouyancyForceVectorVertices;
    private final ArrayList<Vector3f> submergedTriangleVelocityVertices;
    private final ArrayList<Vector3f> principalTriangleVelocityVertices;
    private final ArrayList<Vector3f> resistanceForceVectorVertices;
    private final ArrayList<Vector3f> slammingForceVectorVertices;
    private Vector3f worldPoint;
    private Vector3f linearVelocityStore;
    private Vector3f angularVelocityStore;
    private Vector3f gravity;
    private float gravityScalar;
    private float fluidDensity;
    private float totalSurfaceArea;
    private float totalSubmergedSurfaceArea;
    private float areaThreshold;
    private float[] heightsAboveSurface;
    
    private Vector3f principalA;
    private Vector3f principalB;
    private Vector3f principalC;
    private Vector3f High;
    private Vector3f Mid;
    private Vector3f Low;
    private int h;
    private int m;
    private int l;
    private char first;
    private char second;
    private float h_H;
    private float h_M;
    private float h_L;
    private float t_M;
    private float t_H;
    private float t_L;
    private Vector3f L_M;
    private Vector3f L_H;
    private Vector3f L_J_M;
    private Vector3f L_J_H;
    private Vector3f J_M;
    private Vector3f J_H;
    private Vector3f M_H;
    private Vector3f M_I_M;
    private Vector3f L_I_L;
    private Vector3f I_M;
    private Vector3f I_L;
    
    private Vector3f velocityAtCenterOfGravity;
    private float speedAtCenterOfGravity;
    private Vector3f angularVelocityAtCenterOfGravity;  // angular velocity at centerOfGravity.
    private float boatTriangleArea;
    private float localTriangleArea;
    private Vector3f localTriangleCenter;
    private Vector3f localTriangleNormal;
    private Vector3f velocityAtTriangleCenter;
    private float speedAtTriangleCenter;
    private Vector3f velocityDirectionAtTriangleCenter;
    private float cosTHETAi;
    private float cosTHETAj;
    private Vector3f flowVelocityAtTriangleCenter;
    private float flowSpeedAtTriangleCenter;
    private Vector3f velocityTangentAtTriangleCenter;
    private Vector3f flowTangentDirectionAtTriangleCenter;
    private float coefficientOfFriction;
    private float bodySpeed;
    private float totalFluidTraversalLength;
    private float viscosity; //  0.000001f at 68F, = 0.0000008f at 86F.
    private float reynoldsNumber;
    private float k;
    private float travelFromBow;
    private float bowLocalZ;
    private float aftLocalZ;
    private float triangleSpeedTerm;
    private float pressureLinearDragCoefficient;
    private float pressureQuadraticDragCoefficient;
    private float pressureFalloff;
    private float suctionLinearDragCoefficient;
    private float suctionQuadraticDragCoefficient;
    private float suctionFalloff;
    
    private Vector3f viscousResistanceForce;
    private Vector3f pressureDragForce;
    private Vector3f slammingForce; // FjSlamming
    
    private final float slammingForceRampExponent = 2f; // P (1 gradual, 2 abrupt) 2 is better
    private float GAMMA;
    private float maxGAMMA;
    private float rampedGAMMA;
    //private float boatTriangleArea; // Sbj
    private float submergedTriangleArea; // AjSubmerged(t) ...t = current tick
    private float submergedTriangleAreaPrevious; // AjSubmerged(t-dt) ...t-dt = delta time since last tick, or tpf
    private Vector3f triangleStoppingForce; // FjStopping
    private Vector3f volumeSweptPerSecond;  // VjSwept(t) = AjSubmerged(t) * vi(t)
    private Vector3f volumeSweptPerSecondPrevious; // VjSwept(t-dt) = AjSubmerged(t-dt) * vi(t-dt)
    //private Vector3f triangleNormal; // nj or ni // worldTriangleNormal
    private Vector3f triangleVelocity; // vi(t) // velocityAtTriangleCenter
    private Vector3f triangleVelocityPrevious; // vi(t-dt)
    private Vector3f GAMMAVector;   
    
    // experimental: to compensate for a 0.0000001f dampening jitter in bullet
    //private final float jitterCorrection = 0.9999999f;
    
    private boolean movingForward;
    private boolean turningLeft;
    private boolean movingBackward;
    private boolean turningRight;
    private float boatSpeed;
    private float boatMaxSpeed;
    private float boatTurningSpeed;
    
    private boolean isActive;
    private boolean isGerstnerWave;
    private boolean isDebug;
    
    
        
    public SimpleBoatControl(WaterSurfaceAppState waterSurfaceAppState, RigidBodyControl rigidBodyControl, Geometry geom) {
        this.waterSurfaceAppState = waterSurfaceAppState;
        this.rigidBodyControl = rigidBodyControl;
        this.principalGeom = geom;
        principalMesh = geom.getMesh();
        principalNode = geom.getParent(); // unused
        //System.out.println("principalNode = "+ principalNode.getName());
        
        // setup arrays for vertex and index data of principal Mesh 
        principalVertices = BufferUtils.getVector3Array(principalMesh.getFloatBuffer(VertexBuffer.Type.Position));
        IndexBuffer indexes = principalMesh.getIndexBuffer();
        principalIndices = new int[indexes.size()];
        for(int i=0; i<indexes.size(); i++) { 
            principalIndices[i] = indexes.get(i); 
        }
        
        // get total number of boat triangles
        numTris = indexes.size()/3;
        //System.out.println("Number of Triangles: "+ numTris);
        
        // setup secondary arrays for later use
        principalAreas = new float[numTris];
        submergedPrincipalAreas = new float[numTris];
        submergedPrincipalAreasPrevious = new float[numTris];
        triangleVelocities = new Vector3f[numTris];
        zeroVectorArray(triangleVelocities);
        triangleVelocitiesPrevious = new Vector3f[numTris];
        zeroVectorArray(triangleVelocitiesPrevious);
        
        // setup vertex and index ArrayLists for submerged triangle data
        submergedVertices = new ArrayList<>();
        submergedIndices = new ArrayList<>();
        splitSubVertices = new ArrayList<>();
        splitSubIndices = new ArrayList<>();
        splitSubRejectedVertices = new ArrayList<>(); // for debug rejected mesh
        splitSubRejectedIndices = new ArrayList<>(); // for debug rejected mesh
        
        // setup vertex ArrayList for normals
        splitSubNormalsVertices = new ArrayList<>();
        
        // setup vertex ArrayList for water line data
        waterLineVerts = new ArrayList<>();
        
        // setup vertex ArrayList for triangle center of pressure
        bouyancyForcePointVertices = new ArrayList<>();
        
        // setup vertex ArrayLists for forces applied 
        bouyancyForceVectorVertices = new ArrayList<>();
        submergedTriangleVelocityVertices = new ArrayList<>();
        principalTriangleVelocityVertices = new ArrayList<>();
        resistanceForceVectorVertices = new ArrayList<>();
        slammingForceVectorVertices = new ArrayList<>();

        initializeBoatControl();
    }
    
    private void initializeBoatControl() {
        // setup chasecam
        //setupChaseCam();
        
        // hook into bullet physics space to get update calls
        rigidBodyControl.getPhysicsSpace().addTickListener(this);
        
        // default parameters
        fluidDensity = 999f; // kg/m^3 (water = 999, seawater = 1030)
        boatSpeed = 16f;
        boatMaxSpeed = 64f;
        boatTurningSpeed = 8f;
        
        // Modify for different turning behavior and planing forces
        pressureLinearDragCoefficient = 10f; 
        pressureQuadraticDragCoefficient = 10f; 
        pressureFalloff = 0.3f; // should be smaller than 1
        suctionLinearDragCoefficient = 10f; 
        suctionQuadraticDragCoefficient = 10f; 
        suctionFalloff = 0.3f; // should be smaller than 1
        
        
        // calculate offset of mesh origin to center of Geometry
        // **NOTE -- instead, do this to the mesh before attaching **
//        Vector3f centerOfGravityOffset = Utils.calculateMassCenter(principalMesh);
//        for(Vector3f vertex : principalVertices) { vertex.subtractLocal(centerOfGravityOffset); }
//        principalMesh.setBuffer(VertexBuffer.Type.Position, 3, BufferUtils.createFloatBuffer(principalVertices));
//        principalMesh.updateBound();
        

        // initialize re-used temp variables and constants
        heightsAboveSurface = new float[principalVertices.length];
        areaThreshold = 0.0000001f;
        worldPoint = new Vector3f();
        linearVelocityStore = new Vector3f();
        angularVelocityStore = new Vector3f();
        viscousResistanceForce = new Vector3f();
        pressureDragForce = new Vector3f();
        slammingForce = new Vector3f();
        viscosity = 0.000001f; // 0.000001f at 68F, 0.0000008 at 86F
        gravity = rigidBodyControl.getGravity(); //.mult(jitterCorrection);
        gravityScalar = gravity.getY();
        //gravityScalar = -gravity.length(); // Y component of gravity vector is easier
        //System.out.println("Gravity Vector: "+ gravity);
        
        // setup list of individual triangle areas and calculate total Mesh surface area
        setupPrincipalSurfaceAreas(principalVertices, principalIndices);
                
        // set flag for whether using Gerstner Waves or Sinus Waves
        isGerstnerWave = waterSurfaceAppState.isUsingGerstnerWaves();
        
        // set debug flag
        isDebug = true;
        
        // setup debug Mesh of the currently submerged region of original Mesh
        initDebugSubmergedMesh();
            
        // setup debug Mesh of the current waterline where Mesh intersects surface
        initDebugWaterLine();
        
        // setup debug Mesh of the current submerged region with triangles split horizontally
        initDebugSplitSubMesh();
        
        // setup debug Mesh of the current rejected submerged triangles
        initDebugSplitSubRejectedMesh();
        
        // setup debug Line Mesh of current submerged triangle normals
        initDebugSplitSubNormalsMesh();
        
        // setup debug Line Mesh of current submerged split triangle point of force application
        initDebugForcePointMesh();
        
        // setup debug Line Mesh of current bouyancy force vectors
        initDebugBouyancyForceVectorMesh();
        
        // setup debug Line Mesh of current submerged Triangle velocity vectors
        initDebugSubmergedTriangleVelocityMesh();
        
        // setup debug Line Mesh of current principal Triangle velocity vectors
        initDebugPrincipalTriangleVelocityMesh();
        
        // setup debug Line Mesh of current resistance force vectors
        initDebugResistanceForceVectorMesh();
        
        // setup debug Line Mesh of current slamming force vectors
        initDebugSlammingForceVectorMesh();
        
        // initialize isActive flag (for debug pausing)
        isActive = true;
    }
    
    @Override
    protected void controlUpdate(float tpf) {
        // TODO
    }
    
    @Override
    protected void controlRender(RenderManager rm, ViewPort vp) {
        //Only needed for rendering-related operations,
        //not called when spatial is culled.
    }
    
    @Override
    public SimpleBoatControl cloneForSpatial(Spatial spatial) {
//        Geometry geom;
//        if(spatial instanceof Geometry){
//            geom = (Geometry)spatial;
//        }else{
//            List <Geometry>list = ((Node)spatial).descendantMatches(Geometry.class);
//            if(null != list && list.size() > 0) {
//                geom = list.get(0);
//            }else{
//                geom = principalGeom;
//            }
//        }
//        return new SimpleBoatControl(waterSurfaceAppState, rigidBodyControl, geom);

        throw new UnsupportedOperationException("SimpleBoatControl does not currently support cloneForSpatial().");
    }
    
    @Override
    public void read(JmeImporter im) throws IOException {
        super.read(im);
        InputCapsule in = im.getCapsule(this);
        //TODO: load properties of this Control, e.g.
        //this.value = in.readFloat("name", defaultValue);
    }
    
    @Override
    public void write(JmeExporter ex) throws IOException {
        super.write(ex);
        OutputCapsule out = ex.getCapsule(this);
        //TODO: save properties of this Control, e.g.
        //out.write(this.value, "name", defaultValue);
    }
    
    @Override
    public void prePhysicsTick(PhysicsSpace space, float tpf) {
        if(isActive) {
            computeForces(tpf); // this is where the magic happens
        }
    }

    @Override
    public void physicsTick(PhysicsSpace space, float tpf) {
        // for polling results
    }
    
    private void computeForces(float tpf) {
        // reset lists of vertex data to be reused each tick
        totalSubmergedSurfaceArea = 0;
        submergedVertices.clear();
        submergedIndices.clear();
        waterLineVerts.clear();
        splitSubVertices.clear();
        splitSubIndices.clear();
        splitSubRejectedVertices.clear();
        splitSubRejectedIndices.clear();
        splitSubNormalsVertices.clear();
        bouyancyForcePointVertices.clear();
        bouyancyForceVectorVertices.clear();
        submergedTriangleVelocityVertices.clear();
        principalTriangleVelocityVertices.clear();
        resistanceForceVectorVertices.clear();
        slammingForceVectorVertices.clear();
        //String windingOrder = null;
        
        // calculate current heights above surface for each principal vertex
        calcHeightsAboveSurface();
        
        // move submerged triangles into list, cutting if not fully submerged
        isolateSubmergedTriangles();
        
        // compute and apply Hydrostatic (bouyancy) force on each submerged triangle
        calculateBouyancy(tpf);
        
        // hydrodynamic forces
        calculateResistanceForces(tpf);
        
        
        // update debug meshes
        if(isDebug) {
            updateDebugWaterLine();
            updateDebugSubmergedMesh();
            updateDebugSplitSubMesh();
            //updateDebugSplitSubRejectedMesh();
            updateDebugSplitSubNormalsMesh();
            //updateDebugForcePointMesh();
            //updateDebugBouyancyForceVectorMesh();
            //updateDebugSubmergedTriangleVelocityMesh();
            updateDebugPrincipalTriangleVelocityMesh();
            updateDebugResistanceForceVectorMesh();
            updateDebugSlammingForceVectorMesh();
        }
        
        // debug damping - until all hydrodynamic forces are tuned
        //applyDamping(false);
        
        applyPropulsion(tpf);
    }
    
    private void calculateBouyancy(float tpf) {
        for(int i = 0; i < submergedIndices.size(); i +=3) {
            // for each submerged triangle, clone it's 3 verts, still in local space
            Vector3f p1 = submergedVertices.get(submergedIndices.get(i)).clone();
            Vector3f p2 = submergedVertices.get(submergedIndices.get(i+1)).clone();
            Vector3f p3 = submergedVertices.get(submergedIndices.get(i+2)).clone();
            
            // check for (and skip over) degenerate triangles
            float initialArea = getTriangleArea(p1, p2, p3);
            if(Float.isNaN(initialArea) || initialArea < areaThreshold) {
                //System.out.println("Initial Area["+ initialArea +"], Degenerate Triangle: Skipped");
                continue;
            }
            
            // cut original triangle horizontally into two triangles
            // since the Geom is moving, horizontal is relative to worldSpace Y,
            // points must be provided with respect to world transform of geom
            principalGeom.localToWorld(p1, p1);
            principalGeom.localToWorld(p2, p2);
            principalGeom.localToWorld(p3, p3);
            Vector3f[] splitTris = breakInTwoHorizontally(p1, p2, p3);
            
            // identify points of new horizontal triangles in world space
            Vector3f worldTopApex = splitTris[0];
            Vector3f worldTopBaseA = splitTris[1];
            Vector3f worldTopBaseB = splitTris[2];
            
            Vector3f worldBottomApex = splitTris[3];
            Vector3f worldBottomBaseA = splitTris[4];
            Vector3f worldBottomBaseB = splitTris[5];
            
            // convert horizontal triangle points into local space
            Vector3f localTopApex = new Vector3f();
            Vector3f localTopBaseA = new Vector3f();
            Vector3f localTopBaseB = new Vector3f();
            principalGeom.worldToLocal(worldTopApex, localTopApex);
            principalGeom.worldToLocal(worldTopBaseA, localTopBaseA);
            principalGeom.worldToLocal(worldTopBaseB, localTopBaseB);
            
            Vector3f localBottomApex = new Vector3f();
            Vector3f localBottomBaseA = new Vector3f();
            Vector3f localBottomBaseB = new Vector3f();
            principalGeom.worldToLocal(worldBottomApex, localBottomApex);
            principalGeom.worldToLocal(worldBottomBaseA, localBottomBaseA);
            principalGeom.worldToLocal(worldBottomBaseB, localBottomBaseB);
            
            /* calculate force and it's point of application for each of the 2 triangles */
            
            // get area of each triangle and validate(theoretically, should sum to initialArea)
            float topArea = getTriangleArea(worldTopApex, worldTopBaseA, worldTopBaseB);
            if(Float.isNaN(topArea) || topArea < areaThreshold) {
                //System.out.println("Top Area ["+ topArea +"]: "+ worldTopApex +", "+ worldTopBaseA +", "+ worldTopBaseB);
                topArea = 0f;
            }
            
            float bottomArea = getTriangleArea(worldBottomApex, worldBottomBaseA, worldBottomBaseB);
            if(Float.isNaN(bottomArea) || bottomArea < areaThreshold) {
                //System.out.println("Bottom Area ["+ bottomArea +"]: "+ worldBottomApex +", "+ worldBottomBaseA +", "+ worldBottomBaseB);
                bottomArea = 0f;
            }
            
            
            // For both top and bottom triangles, 
            // validate eligibility (not NaN, has positive area),
            // calculate center of pressure of the triangle
            
    //====== **TOP TRIANGLE** =========================================//
            
            if(topArea > areaThreshold ) { // check for valid area
                totalSubmergedSurfaceArea += topArea; // add to running total
                
                // get top tri normal vector and center point in world space
                Vector3f worldTopNormal = getTriangleNormal(worldTopApex, 
                                                            worldTopBaseA, 
                                                            worldTopBaseB);
                Vector3f worldTopCenter = worldTopApex
                                            .add(worldTopBaseA)
                                            .add(worldTopBaseB)
                                            .mult(FastMath.ONE_THIRD);
                
                // get top tri normal vector and center point in local space
                Vector3f localTopNormal = getTriangleNormal(localTopApex, 
                                                                localTopBaseA, 
                                                                localTopBaseB);
                Vector3f localTopCenter = localTopApex
                                        .add(localTopBaseA)
                                        .add(localTopBaseB)
                                        .mult(FastMath.ONE_THIRD);
                
                // add to split submerged Normals debug mesh
                splitSubNormalsVertices.add(localTopCenter);
                splitSubNormalsVertices.add(localTopCenter.add(localTopNormal.mult(0.33f)));
                
                // get the depth from water of the triangle center in world space
                float topCenterDepth;
                if(!isGerstnerWave) {
                    topCenterDepth = WaterSurfaceGenerator.getHeightAt(
                                        worldTopCenter.getX(), 
                                        worldTopCenter.getZ(), 
                                        waterSurfaceAppState.getElapsedBouyancyTime()) -
                                                worldTopCenter.getY();
                }else{
                    topCenterDepth = WaterSurfaceGenerator.getGerstnerHeightAt(
                                        worldTopCenter.getX(), 
                                        worldTopCenter.getZ(), 
                                        waterSurfaceAppState.getElapsedBouyancyTime()) -
                                                worldTopCenter.getY();
                }
                
                if(topCenterDepth > 0) { // check depth is below surface
                    // calculate the triangle center of pressure in world space
                    Vector3f topCenterOfPressure;
                    if(isHorizontal(worldTopApex, worldTopBaseA, worldTopBaseB)) {
                        topCenterOfPressure = worldTopCenter; // still in world coordinates
                        //System.out.println("Top Triangle is already Horizontal");
                    }else{
                        topCenterOfPressure = getPointOfApplicationTop(
                                worldTopApex, worldTopBaseA, worldTopBaseB); // still in world coordinates
                    }
                    // calculate the bouyancy force in world space, keep only the Y component
                    Vector3f bouyancyForceOnTriangleTop = worldTopNormal.mult(fluidDensity * (gravityScalar*tpf) * topCenterDepth * topArea);
                     bouyancyForceOnTriangleTop.setX(0f);
                     bouyancyForceOnTriangleTop.setZ(0f);
                    //System.out.println("Top force: "+bouyancyForceOnTriangleTop);

                    // apply bouyancy force at center of pressure
                    if(bouyancyForceOnTriangleTop.getY() > 0) { // check force is Up
                        // bring point of application back into Geometry local space
                        principalGeom.worldToLocal(topCenterOfPressure, topCenterOfPressure);// local coordinates
                        
                        // rotate the world force vector into local space
//                        Quaternion inverseRotation = rigidBodyControl.getPhysicsRotation().inverse();
//                        Vector3f localBouyancyForceOnTriangleTop = inverseRotation.mult(bouyancyForceOnTriangleTop, null);
                        
                        // apply the force
                        rigidBodyControl.applyForce(bouyancyForceOnTriangleTop, rigidBodyControl.getPhysicsRotation().mult(topCenterOfPressure));
                        
                        // for debug force mesh
                        bouyancyForceVectorVertices.add(topCenterOfPressure); // still in local
                        Vector3f worldTopCenterOfPressure = new Vector3f();
                        principalGeom.localToWorld(topCenterOfPressure, worldTopCenterOfPressure); // move into world space
                        Vector3f debugForceBegin = bouyancyForceOnTriangleTop.mult(0.01f).negate().add(worldTopCenterOfPressure);
                        principalGeom.worldToLocal(debugForceBegin, debugForceBegin); // bring this new point into local
                        bouyancyForceVectorVertices.add(debugForceBegin);
                    
                    } // else{System.out.println("downward Top force is ignored: "+bouyancyForceOnTriangleTop);}
                    
                    // for debug split submerged mesh
                    addToSplitSubMesh(localTopApex, localTopBaseA, localTopBaseB);
                    
                }else{
                    //System.out.println("Top Triangle above surface skipped, depth: "+ topCenterDepth);
                    // for debug split submerged Rejected mesh
                    addToSplitSubRejectedMesh(localTopApex, localTopBaseA, localTopBaseB);
                } 
                
            } // else{System.out.println("Degenerate Triangle skipped, area: "+ topArea);}
            
    //====== **BOTTOM TRIANGLE** =========================================//
            
            if(bottomArea > areaThreshold ) {
                totalSubmergedSurfaceArea += bottomArea; // add to running total

                Vector3f worldBottomNormal = getTriangleNormal(worldBottomApex, 
                                                            worldBottomBaseA, 
                                                            worldBottomBaseB);
                Vector3f worldBottomCenter = worldBottomApex
                                        .add(worldBottomBaseA)
                                        .add(worldBottomBaseB)
                                        .mult(FastMath.ONE_THIRD);
                
//                Vector3f localBottomNormal = new Vector3f();
//                principalGeom.worldToLocal(worldBottomNormal, localBottomNormal);
//                localBottomNormal.multLocal(FastMath.ONE_THIRD);
//                Vector3f localBottomCenter = new Vector3f();
//                principalGeom.worldToLocal(worldBottomCenter, localBottomCenter);

                Vector3f localBottomNormal = getTriangleNormal(localBottomApex, 
                                                                localBottomBaseA, 
                                                                localBottomBaseB);
                Vector3f localBottomCenter = localBottomApex
                                        .add(localBottomBaseA)
                                        .add(localBottomBaseB)
                                        .mult(FastMath.ONE_THIRD);

                // for debug split submerged Normals mesh
                splitSubNormalsVertices.add(localBottomCenter);
                splitSubNormalsVertices.add(localBottomCenter.add(localBottomNormal.mult(0.33f)));
                
                float bottomCenterDepth;
                if(!isGerstnerWave) {
                    bottomCenterDepth = WaterSurfaceGenerator.getHeightAt(
                                            worldBottomCenter.getX(), 
                                            worldBottomCenter.getZ(), 
                                            waterSurfaceAppState.getElapsedBouyancyTime()) -
                                                    worldBottomCenter.getY();
                }else{
                    bottomCenterDepth = WaterSurfaceGenerator.getGerstnerHeightAt(
                                            worldBottomCenter.getX(), 
                                            worldBottomCenter.getZ(), 
                                            waterSurfaceAppState.getElapsedBouyancyTime()) -
                                                    worldBottomCenter.getY();
                }
                
                if(bottomCenterDepth > 0) {
                    Vector3f bottomCenterOfPressure;     
                    if(isHorizontal(worldBottomApex, worldBottomBaseA, worldBottomBaseB)) {
                        bottomCenterOfPressure = worldBottomCenter;
                    }else{
                        bottomCenterOfPressure = getPointOfApplicationBottom(
                                worldBottomApex, worldBottomBaseA, worldBottomBaseB);
                    }
                    Vector3f bouyancyForceOnTriangleBottom = worldBottomNormal.mult(fluidDensity * (gravityScalar*tpf) * bottomCenterDepth * bottomArea);
                     bouyancyForceOnTriangleBottom.setX(0f);
                     bouyancyForceOnTriangleBottom.setZ(0f);
                    //System.out.println("Bottom force: "+bouyancyForceOnTriangleBottom);

                    if(bouyancyForceOnTriangleBottom.getY() > 0) {
                        // bring point of application back into Geometry local space
                        principalGeom.worldToLocal(bottomCenterOfPressure, bottomCenterOfPressure);
                        
                        // rotate the world force vector into local space
//                        Quaternion inverseRotation = rigidBodyControl.getPhysicsRotation().inverse();
//                        Vector3f localBouyancyForceOnTriangleBottom = inverseRotation.mult(bouyancyForceOnTriangleBottom, null);
                        
                        // apply the force 
                        rigidBodyControl.applyForce(bouyancyForceOnTriangleBottom, rigidBodyControl.getPhysicsRotation().mult(bottomCenterOfPressure));
                      
                        
                        // for debug force mesh
                        bouyancyForceVectorVertices.add(bottomCenterOfPressure);// still in local
                        Vector3f worldBottomCenterOfPressure = new Vector3f();
                        principalGeom.localToWorld(bottomCenterOfPressure, worldBottomCenterOfPressure); // move into world space
                        Vector3f debugForceBegin = bouyancyForceOnTriangleBottom.mult(0.01f).negate().add(worldBottomCenterOfPressure);
                        principalGeom.worldToLocal(debugForceBegin, debugForceBegin); // bring this new point into local
                        bouyancyForceVectorVertices.add(debugForceBegin);
                        
                    } // else{System.out.println("downward Bottom force is ignored: "+bouyancyForceOnTriangleBottom);}
                    
                    // for debug split submerged mesh
                    addToSplitSubMesh(localBottomApex, localBottomBaseA, localBottomBaseB);
                    
                }else{
                    //System.out.println("Bottom Triangle above surface skipped, depth: "+ bottomCenterDepth);
                    // for debug split submerged Rejected mesh
                    addToSplitSubRejectedMesh(localBottomApex, localBottomBaseA, localBottomBaseB);
                } 
                
            } // else{System.out.println("Degenerate Triangle skipped, area: "+ bottomArea);}
            
        } 
        // HydroStatic forces calculated and applied to entire submerged Mesh
    }
    
    private void calculateResistanceForces(float tpf) {
        // reset forces each tick for reuse
        viscousResistanceForce.setX(0f).setY(0f).setZ(0f);
        pressureDragForce.setX(0f).setY(0f).setZ(0f);
        slammingForce.setX(0f).setY(0f).setZ(0f);
        
        // get current position and motion details of body
        velocityAtCenterOfGravity = rigidBodyControl.getLinearVelocity();
        speedAtCenterOfGravity = velocityAtCenterOfGravity.length();
        angularVelocityAtCenterOfGravity = rigidBodyControl.getAngularVelocity();
        
        // get (the z components of) the foremost and rearmost submerged vertices in local space
        //Vector3f localBow = new Vector3f();
        //Vector3f localAft = new Vector3f();
        for(Vector3f point : submergedVertices) { 
            if(point.getZ() > bowLocalZ) { 
                bowLocalZ = point.getZ();
                //localBow = point.clone();
            } 
            if(point.getZ() < aftLocalZ) { 
                aftLocalZ = point.getZ(); 
                //localAft = point.clone(); 
            } 
        } 
        
        // Reynolds number 
        bodySpeed = speedAtCenterOfGravity;
        totalFluidTraversalLength = bowLocalZ - aftLocalZ;
          //System.out.println("Total Fluid Traversal Length: "+ totalFluidTraversalLength);
        reynoldsNumber = (bodySpeed * totalFluidTraversalLength) / viscosity;
          //System.out.println("Reynolds Number: "+ reynoldsNumber);
        
        // "ITTC 1957 model-ship correlation line"
        coefficientOfFriction = 0.075f / FastMath.sqr( FastMath.log(reynoldsNumber, 10f) - 2f ); // once per tick 
          //System.out.println("Coefficient of Friction: "+ coefficientOfFriction);
        
        // over each submerged triangle
        for (int i = 0; i < submergedIndices.size(); i += 3) {
            // get triangle vertex positions in local space
            Vector3f A = submergedVertices.get(submergedIndices.get(i)).clone();
            Vector3f B = submergedVertices.get(submergedIndices.get(i + 1)).clone();
            Vector3f C = submergedVertices.get(submergedIndices.get(i + 2)).clone();
            
            // get triangle vertex positions in world space
            Vector3f worldA = principalGeom.localToWorld(A, null);
            Vector3f worldB = principalGeom.localToWorld(B, null);
            Vector3f worldC = principalGeom.localToWorld(C, null);

            // calculate center, normal, and area of triangle in local space
            localTriangleCenter = getTriangleCenter(A, B, C);
            localTriangleNormal = getTriangleNormal(A, B, C);
            localTriangleArea = getTriangleArea(A, B, C);
            //System.out.println("Triangle "+ i/3 +" Center (Local): "+ localTriangleCenter);
            //System.out.println("Triangle "+ i/3 +" Normal (Local): "+ localTriangleNormal);
            //System.out.println("Triangle "+ i/3 +" Area (Local): "+ localTriangleArea);
            
            // get the center of the triangle relative to the rigidBody rotation
            Vector3f relativeTriangleCenter = rigidBodyControl.getPhysicsRotation().mult(localTriangleCenter);
            //System.out.println("Triangle "+ i/3 +" Center (Relative): "+ relativeTriangleCenter);
            
            // calculate center and normal of triangle in world space
            Vector3f worldTriangleCenter = getTriangleCenter(worldA, worldB, worldC);
            Vector3f worldTriangleNormal = getTriangleNormal(worldA, worldB, worldC);
            //System.out.println("Triangle "+ i/3 +" Center (World): "+ worldTriangleCenter);
            //System.out.println("Triangle "+ i/3 +" Normal (World): "+ worldTriangleNormal);
            
            // calculate triangle velocity data in local coordinates relative to physics rotation
            //velocityAtTriangleCenter = velocityAtCenterOfGravity.add(angularVelocityAtCenterOfGravity.cross(relativeTriangleCenter)); 
            velocityAtTriangleCenter = velocityAtCenterOfGravity.
                    add(angularVelocityAtCenterOfGravity.
                            cross(worldTriangleCenter.subtract(
                                    rigidBodyControl.getPhysicsLocation()))); 
            speedAtTriangleCenter = velocityAtTriangleCenter.length();
            velocityDirectionAtTriangleCenter = velocityAtTriangleCenter.normalize();
            //System.out.println("Triangle "+ i/3 +" Velocity: "+ velocityAtTriangleCenter);
            //System.out.println("Triangle "+ i/3 +" Speed: "+ speedAtTriangleCenter);
            
            // debug mesh - displays submerged triangle velocity direction lines
            Vector3f debugVelocityForceEnd = rigidBodyControl.getPhysicsRotation().inverse().mult(velocityAtTriangleCenter).mult(0.33f).add(localTriangleCenter);
            submergedTriangleVelocityVertices.add(debugVelocityForceEnd);
            submergedTriangleVelocityVertices.add(localTriangleCenter);

            // calculate relative flow velocity at triangle center ?
            //velocityTangentAtTriangleCenter = worldTriangleNormal.cross(velocityAtTriangleCenter.cross(worldTriangleNormal).divide(worldTriangleNormal.length())).divide(worldTriangleNormal.length());
            velocityTangentAtTriangleCenter = worldTriangleNormal.cross(velocityAtTriangleCenter.cross(worldTriangleNormal));
            flowTangentDirectionAtTriangleCenter = velocityTangentAtTriangleCenter.negate().normalize(); 
            flowVelocityAtTriangleCenter = flowTangentDirectionAtTriangleCenter.mult(speedAtTriangleCenter); 
            flowSpeedAtTriangleCenter = speedAtTriangleCenter;
            
        // ** VISCOUS RESISTANCE FORCE **

            // calculate distance flow has traveled from bow to triangleCenter
            travelFromBow = bowLocalZ - localTriangleCenter.getZ();
            // validate/clamp to be less than the the total submerged fluid traversal length
            if(travelFromBow > totalFluidTraversalLength) {
                travelFromBow = totalFluidTraversalLength;
            }
            //System.out.println("Triangle "+ i/3 +" Travel from Bow: "+ travelFromBow +" / "+ totalFluidTraversalLength);
            k = Utils.mapToRange(0f, totalFluidTraversalLength, -1f, 1f, travelFromBow);
                if(Float.isNaN(k) || Float.isInfinite(k)) {
                    k = 0f; // so (1 + k) becomes 1f
                }
            //System.out.println("Triangle "+ i/3 +" (1 + k) factor k: "+ k);
            
            // determine direction and magnitude of resistance force
            viscousResistanceForce = flowVelocityAtTriangleCenter.mult(
                    0.5f * fluidDensity * (1 + k) * coefficientOfFriction
                    * localTriangleArea * flowSpeedAtTriangleCenter);
            
            if (isValidForce(viscousResistanceForce)) {
                rigidBodyControl.applyForce(viscousResistanceForce.mult(tpf), 
                        rigidBodyControl.getPhysicsRotation().mult(localTriangleCenter));
                
                // Debug Mesh points
                Vector3f debugResistanceForceBegin = viscousResistanceForce.mult(-tpf).add(worldTriangleCenter);
                principalGeom.worldToLocal(debugResistanceForceBegin, debugResistanceForceBegin);
                resistanceForceVectorVertices.add(debugResistanceForceBegin);
                resistanceForceVectorVertices.add(localTriangleCenter);
                
            } else {
                //System.out.println("Viscous Resistance skipped for triangle " + i / 3);
            }
        
        // ** PRESSURE DRAG FORCE **
        
//            System.out.println("--Triangle "+ i/3 +" Pressure Drag Forces--");
              //System.out.println("Area: "+ localTriangleArea);
            
            // A reference speed used when modifying the parameters
            //float referenceSpeed = boatMaxSpeed;
            float referenceSpeed = speedAtTriangleCenter; // to ensure a speed term of 1f
            
            triangleSpeedTerm = speedAtTriangleCenter / referenceSpeed;
              //System.out.println("Velocity Scalar: "+ triangleSpeedTerm);
            
            cosTHETAi = velocityDirectionAtTriangleCenter.dot(worldTriangleNormal);
              //System.out.println("cosTHETAi: "+ cosTHETAi);
            
            
            // determine the drag force for pressure or suction
            float dragForceMagnitude;
            if (cosTHETAi >= 0f) {
                  //System.out.println("Dragtype: Pressure");
                dragForceMagnitude = ((pressureLinearDragCoefficient * triangleSpeedTerm) + 
                        (pressureQuadraticDragCoefficient * (triangleSpeedTerm * triangleSpeedTerm))) * 
                        localTriangleArea * FastMath.pow(cosTHETAi, pressureFalloff);
//                  System.out.println("((CPD_1:"+ pressureLinearDragCoefficient +" * v:"+ triangleSpeedTerm +") + (CPD_2:"+ pressureQuadraticDragCoefficient +" * (v:"+ triangleSpeedTerm +" * v:"+ triangleSpeedTerm +"))) * Area:"+ localTriangleArea +" * cosTheta["+ cosTHETAi +"]to the Fp["+ pressureFalloff +"]:"+ FastMath.pow(cosTHETAi, pressureFalloff));
                  //System.out.println("pressureLinearDragCoefficient: "+ pressureLinearDragCoefficient);
                  //System.out.println("pressureQuadraticDragCoefficient: "+ pressureQuadraticDragCoefficient);
                  //System.out.println("pressureFalloff: "+ pressureFalloff);
//                  System.out.println("Drag Force Magnitude: "+ dragForceMagnitude);
//                  System.out.println("Normal: "+ worldTriangleNormal);
                    
                pressureDragForce = worldTriangleNormal.mult(-dragForceMagnitude);
//                  System.out.println("Pressure Drag Force: "+ pressureDragForce);
            } else {
                  //System.out.println("Dragtype: Suction");
                dragForceMagnitude =  ((suctionLinearDragCoefficient * triangleSpeedTerm) + 
                        (suctionQuadraticDragCoefficient * (triangleSpeedTerm * triangleSpeedTerm))) * 
                        localTriangleArea * -FastMath.pow(-cosTHETAi, suctionFalloff); // double negatives?
//                  System.out.println("((CSD_1:"+ suctionLinearDragCoefficient +" * v:"+ triangleSpeedTerm +") + (CSD_2:"+ suctionQuadraticDragCoefficient +" * (v:"+ triangleSpeedTerm +" * v:"+ triangleSpeedTerm +"))) * Area:"+ localTriangleArea +" * cosTheta["+ cosTHETAi +"]to the Fs["+ suctionFalloff +"]:"+ -FastMath.pow(-cosTHETAi, suctionFalloff));
                  //System.out.println("suctionLinearDragCoefficient: "+ suctionLinearDragCoefficient);
                  //System.out.println("suctionQuadraticDragCoefficient: "+ suctionQuadraticDragCoefficient);
                  //System.out.println("suctionFalloff: "+ suctionFalloff);
//                  System.out.println("Drag Force Magnitude: "+ dragForceMagnitude);
//                  System.out.println("Normal: "+ worldTriangleNormal);
                  
                pressureDragForce = worldTriangleNormal.mult(dragForceMagnitude);
//                  System.out.println("Suction Drag Force: "+ pressureDragForce);
            }
            
            if (isValidForce(pressureDragForce)) {
                //rigidBodyControl.applyForce(pressureDragForce.mult(tpf), rigidBodyControl.getPhysicsRotation().mult(localTriangleCenter));
                rigidBodyControl.applyForce(pressureDragForce.mult(tpf), relativeTriangleCenter);
                
                // Debug Mesh points
                Vector3f debugResistanceForceBegin = pressureDragForce.mult(-tpf).add(worldTriangleCenter);
                principalGeom.worldToLocal(debugResistanceForceBegin, debugResistanceForceBegin);
                resistanceForceVectorVertices.add(debugResistanceForceBegin);
                resistanceForceVectorVertices.add(localTriangleCenter);
                // or //
//                Vector3f debugResistanceForceBegin = rigidBodyControl.getPhysicsRotation().inverse().mult(pressureDragForce).mult(-tpf).add(localTriangleCenter);
//                resistanceForceVectorVertices.add(debugResistanceForceBegin);
//                resistanceForceVectorVertices.add(localTriangleCenter);
            } else {
                System.out.println("Pressure Drag skipped for triangle " + i / 3);
            }
            
        } // over each Submerged Triangle
        
        // initialize reused vectors
        //volumeSweptPerSecond = new Vector3f();
        //volumeSweptPerSecondPrevious = new Vector3f();
        //triangleStoppingForce = new Vector3f();
        //slammingForce = new Vector3f();
        
        
        
        // Slamming Force:
        //  must be calculated AFTER all other resistance forces
        arrayIndex = 0;
        //zeroVectorArray(triangleVelocities);
        for(int i = 0; i < principalIndices.length; i += 3) {
                System.out.println("Slamming Index: "+ arrayIndex);
            
            Vector3f A = principalVertices[principalIndices[i]];
            Vector3f B = principalVertices[principalIndices[i+1]];
            Vector3f C = principalVertices[principalIndices[i+2]];
            
            Vector3f worldA = principalGeom.localToWorld(A, null);
            Vector3f worldB = principalGeom.localToWorld(B, null);
            Vector3f worldC = principalGeom.localToWorld(C, null);
            
            boatTriangleArea = principalAreas[arrayIndex];
            submergedTriangleArea = submergedPrincipalAreas[arrayIndex];
            submergedTriangleAreaPrevious = submergedPrincipalAreasPrevious[arrayIndex];
                //System.out.println("boatTriangleArea: "+ boatTriangleArea);
                //System.out.println("submergedTriangleArea: "+ submergedTriangleArea);
                //System.out.println("submergedTriangleAreaPrevious: "+ submergedTriangleAreaPrevious);
            
            Vector3f principalTriangleCenter = getTriangleCenter(A, B, C);
            Vector3f worldTriangleCenter = getTriangleCenter(worldA, worldB, worldC);
            Vector3f relativeTriangleCenter = rigidBodyControl.getPhysicsRotation().mult(principalTriangleCenter);
            //Vector3f principalTriangleNormal = getTriangleNormal(A, B, C);
            Vector3f worldTriangleNormal = getTriangleNormal(worldA, worldB, worldC);
                //System.out.println("principalTriangleCenter: "+ principalTriangleCenter);
                //System.out.println("worldTriangleCenter: "+ worldTriangleCenter);
                //System.out.println("relativeTriangleCenter: "+ relativeTriangleCenter);
                //System.out.println("worldTriangleNormal: "+ worldTriangleNormal);

            // get velocity at center of triangle // world coords?
            triangleVelocity = velocityAtCenterOfGravity.
                    add(angularVelocityAtCenterOfGravity.
                            cross(worldTriangleCenter.subtract(
                                    rigidBodyControl.getPhysicsLocation()))); 
            triangleVelocities[arrayIndex] = triangleVelocity; // add to array for slamming forces
            triangleVelocityPrevious = triangleVelocitiesPrevious[arrayIndex];
                //System.out.println("triangleVelocity: "+ triangleVelocity);
                //System.out.println("triangleVelocityPrevious: "+ triangleVelocityPrevious);

            // debug mesh - displays principal triangle velocity direction lines
            Vector3f debugVelocityForceEnd = rigidBodyControl.getPhysicsRotation().inverse().mult(triangleVelocity).mult(0.33f).add(principalTriangleCenter);
            principalTriangleVelocityVertices.add(debugVelocityForceEnd);
            principalTriangleVelocityVertices.add(principalTriangleCenter);
            
            // calculate slamming force only on principal triangles with some area submerged 
            if(submergedTriangleArea > 0f) {
                volumeSweptPerSecond = triangleVelocity.mult(submergedTriangleArea);
                volumeSweptPerSecondPrevious = triangleVelocityPrevious.mult(submergedTriangleAreaPrevious);
                    //System.out.println("volumeSweptPerSecond: "+ volumeSweptPerSecond);
                    //System.out.println("volumeSweptPerSecondPrevious: "+ volumeSweptPerSecondPrevious);
                
                Vector3f deltaVolume = volumeSweptPerSecond.subtract(volumeSweptPerSecondPrevious);
                float divisor = boatTriangleArea * tpf;
                    //System.out.println("deltaVolume: "+ deltaVolume);
                    //System.out.println("boatTriangleArea * tpf: "+ divisor);

                GAMMAVector = deltaVolume.divide(boatTriangleArea * tpf);
                GAMMA = GAMMAVector.length();
                    //System.out.println("GAMMAVector: "+ GAMMAVector);
                    //System.out.println("GAMMA: "+ GAMMA);

                //cosTHETAj = triangleVelocity.normalize().dot(worldTriangleNormal); // normalize velocity
                cosTHETAj = triangleVelocity.dot(worldTriangleNormal);
                System.out.println("cosTHETAj: "+ cosTHETAj);

                if(cosTHETAj > 0f) {
                    float mass = rigidBodyControl.getMass();
                        //System.out.println("mass: "+ mass);
                    //triangleStoppingForce = triangleVelocity.negate().mult(mass * ((2 * submergedTriangleArea) / totalSurfaceArea ));
                    triangleStoppingForce = triangleVelocity.mult(mass * ((2 * submergedTriangleArea) / totalSurfaceArea ));
                    //maxGAMMA = triangleStoppingForce.length();
                    maxGAMMA = GAMMA; // ???
                        System.out.println("triangleStoppingForce: "+ triangleStoppingForce);
                        //System.out.println("maxGAMMA: "+ maxGAMMA);

                    if(GAMMA > maxGAMMA){
                        //System.out.println("GAMMA too high, traingleStoppingForce applied");
                        slammingForce = triangleStoppingForce.negate();
                            System.out.println("slammingForce (stoppingForce): "+ triangleStoppingForce);
                    }else{
                        float slammingCheat = 100f;
                        float clampedGAMMA = GAMMA / maxGAMMA;
                        rampedGAMMA = FastMath.pow(clampedGAMMA, slammingForceRampExponent);
                        slammingForce = triangleStoppingForce.mult(cosTHETAj).mult(rampedGAMMA).negate();
                        //slammingForce = triangleStoppingForce.mult(cosTHETAj).mult(rampedGAMMA).mult(slammingCheat);
                            //System.out.println("clampedGAMMA: "+ clampedGAMMA(GAMMA, maxGAMMA));
                            //System.out.println("slammingForceRampExponent: "+ slammingForceRampExponent);
                            //System.out.println("rampedGAMMA: "+ rampedGAMMA);
                            System.out.println("slammingForce: "+ slammingForce);
                    }

                    // slamming force applied to triangle
                    //rigidBodyControl.applyForce(slammingForce, rigidBodyControl.getPhysicsRotation().mult(localTriangleCenter));
                    rigidBodyControl.applyForce(slammingForce, principalTriangleCenter);

                    // for debug force mesh
                    Vector3f debugForceBegin = slammingForce.negate().add(worldTriangleCenter);
                    principalGeom.worldToLocal(debugForceBegin, debugForceBegin); // bring this new point into local
                    slammingForceVectorVertices.add(debugForceBegin);
                    slammingForceVectorVertices.add(principalTriangleCenter);

                }else{
                    System.out.println("cosTHETAj is negative, slamming force skipped");
                }
            }else{ 
                //System.out.println("Triangle is not submerged, slamming force skipped");
            }
            
            arrayIndex++;
            
        } // over each principal triangle -Slamming Force
        
        submergedPrincipalAreasPrevious = submergedPrincipalAreas.clone();
        triangleVelocitiesPrevious = triangleVelocities.clone();
        
    } // every tick
    
    private void applyPropulsion(float tpf) {
        // general force at center of gravity, needs improvement
        if(movingForward && submergedVertices.size() > 0) {
            rigidBodyControl.applyCentralForce(
                    rigidBodyControl.getPhysicsRotation().mult(Vector3f.UNIT_Z)
                            //.setY(0f).normalize() // to keep horizontal
                            .mult(boatSpeed / (rigidBodyControl.getMass()*0.000025f)));
        }
        
        if(movingBackward && submergedVertices.size() > 0) {
            rigidBodyControl.applyCentralForce(
                    rigidBodyControl.getPhysicsRotation().mult(Vector3f.UNIT_Z)
                            //.setY(0f).normalize() // to keep horizontal
                            .mult(-boatSpeed/2f / (rigidBodyControl.getMass()*0.000025f)));
        }
        
        if(turningLeft && submergedVertices.size() > 0) {
            rigidBodyControl.applyTorque(
                    rigidBodyControl.getPhysicsRotation().mult(Vector3f.UNIT_Y)
                            //Vector3f.UNIT_Y // to keep horizontal
                            .mult(boatTurningSpeed  / (rigidBodyControl.getMass()*0.00005f)));
        }
        
        if(turningRight && submergedVertices.size() > 0) {
            rigidBodyControl.applyTorque(
                    rigidBodyControl.getPhysicsRotation().mult(Vector3f.UNIT_Y)
                            //Vector3f.UNIT_Y // to keep horizontal
                            .mult(-boatTurningSpeed  / (rigidBodyControl.getMass()*0.00005f)));
        }
    }
    
    private void calcHeightsAboveSurface() {
        // height above water surface, negative if submerged (hullVertex.y - waterSurfaceHeight)
        // setup list of signed distance from water surface for each vertex
        // calculated considering the Geometry's world translation/rotation.
        for (int i = 0; i < heightsAboveSurface.length; i++) {
            worldPoint = principalGeom.localToWorld(principalVertices[i], worldPoint);
            if(!isGerstnerWave) {
                heightsAboveSurface[i] = worldPoint.getY() - 
                        WaterSurfaceGenerator.getHeightAt(
                            worldPoint.getX(), 
                            worldPoint.getZ(), 
                            waterSurfaceAppState.getElapsedBouyancyTime());
            }else{
                heightsAboveSurface[i] = worldPoint.getY() -
                        WaterSurfaceGenerator.getGerstnerHeightAt(
                            worldPoint.getX(), 
                            worldPoint.getZ(), 
                            waterSurfaceAppState.getElapsedBouyancyTime());
            }
        }
    }
    
    private void isolateSubmergedTriangles() {
        arrayIndex = 0;
        int vertsUnder;
        int vertIndex = 0;
        for(int i = 0; i < principalIndices.length; i += 3){
            // get count of submerged principal triangle vertices (0,1,2,3)
            vertsUnder = 0;
            if(heightsAboveSurface[principalIndices[i+0]] < 0) {vertsUnder++;}
            if(heightsAboveSurface[principalIndices[i+1]] < 0) {vertsUnder++;}
            if(heightsAboveSurface[principalIndices[i+2]] < 0) {vertsUnder++;}
            
            // 0 submerged triangle verts - omit triangle
            if(vertsUnder == 0) {
                //System.out.println("0 under - triangle omitted");
                
                // set submerged area to be 0f in array -for slamming force
                submergedPrincipalAreas[arrayIndex] = 0f;
                
                arrayIndex++;
                continue;
            }
            
            // 3 submerged triangle verts - original triangle is added
            if(vertsUnder == 3) {
                //System.out.println("3 under - triangle preserved");
                submergedVertices.add(principalVertices[principalIndices[i+0]].clone());
                submergedVertices.add(principalVertices[principalIndices[i+1]].clone());
                submergedVertices.add(principalVertices[principalIndices[i+2]].clone());
                submergedIndices.add(vertIndex+0);
                submergedIndices.add(vertIndex+1);
                submergedIndices.add(vertIndex+2);
                
                // set submerged area to entire principal triangle area for slamming force
                submergedPrincipalAreas[arrayIndex] = principalAreas[arrayIndex];
                
                vertIndex += 3;
                arrayIndex++;
                continue;
            }
            
            // if all triangle verts not above or below,
            // determine High, Mid, and Low verts relative to surface
            if(heightsAboveSurface[principalIndices[i+0]] > heightsAboveSurface[principalIndices[i+1]]) {
                if(heightsAboveSurface[principalIndices[i+1]] > heightsAboveSurface[principalIndices[i+2]]) {
                    High = principalVertices[principalIndices[i+0]].clone();
                    Mid = principalVertices[principalIndices[i+1]].clone();
                    Low = principalVertices[principalIndices[i+2]].clone();
                    h = 0;
                    m = 1;
                    l = 2;
                    first = 'h';
                    second = 'm';
                    //third = 'l';
                    //windingOrder = "HML";
                }else if(heightsAboveSurface[principalIndices[i+0]] > heightsAboveSurface[principalIndices[i+2]]) {
                    High = principalVertices[principalIndices[i+0]].clone();
                    Mid = principalVertices[principalIndices[i+2]].clone();
                    Low = principalVertices[principalIndices[i+1]].clone();
                    h = 0;
                    m = 2;
                    l = 1;
                    first = 'h';
                    second = 'l';
                    //third = 'm';
                    //windingOrder = "HLM";
                }else{
                    High = principalVertices[principalIndices[i+2]].clone();
                    Mid = principalVertices[principalIndices[i+0]].clone();
                    Low = principalVertices[principalIndices[i+1]].clone();
                    h = 2;
                    m = 0;
                    l = 1;
                    first = 'm';
                    second = 'l';
                    //third = 'h';
                    //windingOrder = "MLH";
                }
            }else{
                if(heightsAboveSurface[principalIndices[i+0]] > heightsAboveSurface[principalIndices[i+2]]) {
                    High = principalVertices[principalIndices[i+1]].clone();
                    Mid = principalVertices[principalIndices[i+0]].clone();
                    Low = principalVertices[principalIndices[i+2]].clone();
                    h = 1;
                    m = 0;
                    l = 2;
                    first = 'm';
                    second = 'h';
                    //third = 'l';
                    //windingOrder = "MHL";
                }else if(heightsAboveSurface[principalIndices[i+1]] > heightsAboveSurface[principalIndices[i+2]]) {
                    High = principalVertices[principalIndices[i+1]].clone();
                    Mid = principalVertices[principalIndices[i+2]].clone();
                    Low = principalVertices[principalIndices[i+0]].clone();
                    h = 1;
                    m = 2;
                    l = 0;
                    first = 'l';
                    second = 'h';
                    //third = 'm';
                    //windingOrder = "LHM";
                }else{
                    High = principalVertices[principalIndices[i+2]].clone();
                    Mid = principalVertices[principalIndices[i+1]].clone();
                    Low = principalVertices[principalIndices[i+0]].clone();
                    h = 2;
                    m = 1;
                    l = 0;
                    first = 'l';
                    second = 'm';
                    //third = 'h';
                    //windingOrder = "LMH";
                }
            } 
            // H, M, L are determined, and 
            // original winding order is preserved
            
            // get the heights from water for each vert respectively
            h_H = heightsAboveSurface[principalIndices[i+h]];
            h_M = heightsAboveSurface[principalIndices[i+m]];
            h_L = heightsAboveSurface[principalIndices[i+l]];
            
        // 1 submerged triangle vertex - 1 new triangle is added
            if(vertsUnder == 1) {
                t_M = -h_L/(h_M - h_L);
                t_H = -h_L/(h_H - h_L);
                L_M = Mid.subtract(Low);
                L_H = High.subtract(Low);
                L_J_M = L_M.mult(t_M);
                L_J_H = L_H.mult(t_H);
                
                J_M = Low.add(L_J_M);
                J_H = Low.add(L_J_H);
                
                // add the two new verts to the waterline mesh vertex list
                waterLineVerts.add(J_M.clone());
                waterLineVerts.add(J_H.clone());
                
                // add the new triangle to submerged triangle list
                // *must determine winding order*
                
                // (H)\----/(M)
                //  J_H\--/J_M
                //      \/
                //       L
                if(first == 'l') {
                    if(second == 'h') { // LHM  
                        submergedVertices.add(Low.clone());
                        submergedVertices.add(J_H.clone());
                        submergedVertices.add(J_M.clone());
                    }else { // second == 'm' // LMH 
                        submergedVertices.add(Low.clone());
                        submergedVertices.add(J_M.clone());
                        submergedVertices.add(J_H.clone());
                    }
                }else if(first == 'm') {
                    if(second == 'l') { // MLH  
                        submergedVertices.add(J_M.clone());
                        submergedVertices.add(Low.clone());
                        submergedVertices.add(J_H.clone());
                    }else { // second == 'h' // MHL 
                        submergedVertices.add(J_M.clone());
                        submergedVertices.add(J_H.clone());
                        submergedVertices.add(Low.clone());
                    }
                }else { // first == 'h'
                    if(second == 'm') { // HML  
                        submergedVertices.add(J_H.clone());
                        submergedVertices.add(J_M.clone());
                        submergedVertices.add(Low.clone());
                    }else { // second == 'l' // HLM 
                        submergedVertices.add(J_H.clone());
                        submergedVertices.add(Low.clone());
                        submergedVertices.add(J_M.clone());
                    }
                }               

                submergedIndices.add(vertIndex+0);
                submergedIndices.add(vertIndex+1);
                submergedIndices.add(vertIndex+2);
                
                // set submerged area to new triangle area in array for slamming force
                submergedPrincipalAreas[arrayIndex] = getTriangleArea(J_H, J_M, Low);
                
                vertIndex += 3;
                arrayIndex++;
                //System.out.println("1 under - "+ J_M +" ->"+ J_H);
                continue;
            }
            
        // 2 sumberged triangle principalVertices - 2 new triangles added
            if(vertsUnder == 2) { 
                t_M = -h_M/(h_H - h_M);
                t_L = -h_L/(h_H - h_L);
                M_H = High.subtract(Mid);
                L_H = High.subtract(Low);
                M_I_M = M_H.mult(t_M);
                L_I_L = L_H.mult(t_L);
                
                I_M = Mid.add(M_I_M);
                I_L = Low.add(L_I_L);
                
                // add the two new verts to the waterline mesh vertex list
                waterLineVerts.add(I_M.clone());
                waterLineVerts.add(I_L.clone());
                
                // add 2 new triangles to submerged triangle list
                // *must determine winding order*
                
                //     (H)
                //     /\
                // I_M/--\I_L
                //  M/____\L
                if(first == 'l') {
                    if(second == 'h') { // LHM counter-clockwise
                        submergedVertices.add(Low.clone());
                        submergedVertices.add(I_L.clone());
                        submergedVertices.add(Mid.clone());
                        
                        submergedVertices.add(Mid.clone());
                        submergedVertices.add(I_L.clone());
                        submergedVertices.add(I_M.clone());
                    }else { // second == 'm' // LMH clockwise
                        submergedVertices.add(Low.clone());
                        submergedVertices.add(Mid.clone());
                        submergedVertices.add(I_L.clone());
                        
                        submergedVertices.add(I_L.clone());
                        submergedVertices.add(Mid.clone());
                        submergedVertices.add(I_M.clone());
                    }
                }else if(first == 'm') {
                    if(second == 'l') { // MLH counter-clockwise
                        submergedVertices.add(Mid.clone());
                        submergedVertices.add(Low.clone());
                        submergedVertices.add(I_M.clone());
                        
                        submergedVertices.add(I_M.clone());
                        submergedVertices.add(Low.clone());
                        submergedVertices.add(I_L.clone());
                    }else { // second == 'h' // MHL clockwise
                        submergedVertices.add(Mid.clone());
                        submergedVertices.add(I_M.clone());
                        submergedVertices.add(Low.clone());
                        
                        submergedVertices.add(Low.clone());
                        submergedVertices.add(I_M.clone());
                        submergedVertices.add(I_L.clone());
                    }
                }else { // first == 'h'
                    if(second == 'm') { // HML counter-clockwise
                        submergedVertices.add(I_L.clone());
                        submergedVertices.add(I_M.clone());
                        submergedVertices.add(Low.clone());
                        
                        submergedVertices.add(Low.clone());
                        submergedVertices.add(I_M.clone());
                        submergedVertices.add(Mid.clone());
                    }else { // second == 'l' // HLM clockwise
                        submergedVertices.add(I_M.clone());
                        submergedVertices.add(I_L.clone());
                        submergedVertices.add(Mid.clone());
                        
                        submergedVertices.add(Mid.clone());
                        submergedVertices.add(I_L.clone());
                        submergedVertices.add(Low.clone());
                    }
                }
                
                submergedIndices.add(vertIndex+0);
                submergedIndices.add(vertIndex+1);
                submergedIndices.add(vertIndex+2);
                
                submergedIndices.add(vertIndex+3);
                submergedIndices.add(vertIndex+4);
                submergedIndices.add(vertIndex+5);
                
                // set submerged area to be the sum of 
                //  both new triangle areas in array for slamming force
                float firstArea = getTriangleArea(I_M, Mid, I_L);
                float secondArea = getTriangleArea(Low, I_L, Mid);
                submergedPrincipalAreas[arrayIndex] = firstArea + secondArea;
                
                vertIndex += 6f;
                arrayIndex++;
                //System.out.println("2 under - "+ I_M +" ->"+ I_L);
                //continue;
            }
            
        } // lists of submerged verts and their indices populated
        
    }
    
    private Vector3f[] breakInTwoHorizontally(Vector3f p1, Vector3f p2, Vector3f p3) {
        /* Expectation is that points are given in worldSpace and in order of triangle winding */
        
        Vector3f[] splitTris = new Vector3f[6]; // return array
        Vector3f high; // high
        Vector3f mid; // mid -- this height will equal height of N
        Vector3f low; // low
        Vector3f N = new Vector3f(); // new point cutting triangle in two horizontally (along with mid)
        
        // return an array of 6 vectors, 
        // where first 3 make up the top triangle (apex up), 
        // and the second 3 make up the bottom triangle (apex down),
        // with each triangle beginning its vertex winding with
        // the apex vertex opposite of its horizontal base, 
        // whether up or down
        // Forced indexing -> splitTris[0] == upApex && splitTris[3] == downApex
        
        // HML     or     HLM
        // HMN, LNM    HNM, LMN
        //    H,       H,
        //    /|        |\
        //   / |        | \
        // M/..|N      N|..\M
        //  \  | < or > |  /
        //   \ |        | /
        //    \|        |/
        //    L'       L'
        // Triangle Winding
        // (upApex, Lbase, Rbase)
        // (downApex, Rbase, Lbase)
        
        // initialize the return array with all zero vector (Vector3f(0f,0f,0f))
        Vector3f zeroVec = new Vector3f(0,0,0);
        for(int i = 0; i < 6; i++) {
            splitTris[i] = zeroVec;
        }
        
        /* Step 1 - Check for the case of triangle already being horizontal */
        
        // If already fully horizontal:
        // * original order is preserved
        // * no Forced Apex indexing required
        // * triangle centroid will be used as center of pressure
        if(isHorizontal(new Triangle(p1, p2, p3))) {
            splitTris[0] = p1;
            splitTris[1] = p2;
            splitTris[2] = p3;
            return splitTris;
        }
        
        // If triangle already has a horizontal edge:
        // * determine whether it is a top tri or bottom tri,
        // * update only those array indices (leaving the other zero vectors), 
        // * order them according to winding order and Forced Apex indexing,
        // * immediately return array
        if(p1.getY() == p2.getY()) {
            if(p1.getY() < p3.getY()) { // p3 = upApex
                splitTris[0] = p3.clone();
                splitTris[1] = p1.clone();
                splitTris[2] = p2.clone();
                return splitTris;
            }else{                      // p3 = downApex
                splitTris[3] = p3.clone();
                splitTris[4] = p1.clone();
                splitTris[5] = p2.clone();
                return splitTris;
            }
        }else if(p2.getY() == p3.getY()) {
            if(p2.getY() < p1.getY()) { // p1 = upApex
                splitTris[0] = p1.clone();
                splitTris[1] = p2.clone();
                splitTris[2] = p3.clone();
                return splitTris;
            }else{                      // p1 = downApex
                splitTris[3] = p1.clone();
                splitTris[4] = p2.clone();
                splitTris[5] = p3.clone();
                return splitTris;
            }
        }else if(p3.getY() == p1.getY()) {
             if(p3.getY() < p2.getY()) { // p2 = upApex
                splitTris[0] = p2.clone();
                splitTris[1] = p3.clone();
                splitTris[2] = p1.clone();
                return splitTris;
            }else{                      // p2 = downApex
                splitTris[3] = p2.clone();
                splitTris[4] = p3.clone();
                splitTris[5] = p1.clone();
                return splitTris;
            }
        }
        // Pre-existing horizontal condition has been checked
        
        
        /* Step 2 - Find the single point that cuts the triangle in 2 Horizontally */
        
        // Re-sort vertices according to height (compare Y components), 
        // preserving original triangle winding order
        char point1; // tag for first winding point
        char point2; // tag for second winding point
        //char point3; // implied from 1 and 2 (can be only one left)
        if(p1.getY() > p2.getY()) {
            // p1 > p2
            if(p2.getY() > p3.getY()) {
                // p1 > p2 > p3
                high = p1;
                mid = p2;
                low = p3; 
                point1 = 'h';
                point2 = 'm';
            }else{
                // (p1, p3) > p2
                if(p1.getY() > p3.getY()) {
                    // p1 > p3 > p2
                    high = p1;
                    mid = p3;
                    low = p2;
                    point1 = 'h';
                    point2 = 'l';
                }else{
                    // p3 > p1 > p2
                    high = p3;
                    mid = p1;
                    low = p2;
                    point1 = 'm';
                    point2 = 'l';
                }
            }
        }else{
            // p2 > p1
            if(p3.getY() > p2.getY()) {
                // p3 > p2 > p1
                high = p3;
                mid = p2;
                low = p1;
                point1 = 'l';
                point2 = 'm';
            }else{
                // p2 > (p3, p1)
                if(p1.getY() > p3.getY()) {
                    // p2 > p1 > p3
                    high = p2;
                    mid = p1;
                    low = p3;
                    point1 = 'm';
                    point2 = 'h';
                }else{
                    // p2 > p3 > p1
                    high = p2;
                    mid = p3;
                    low = p1;
                    point1 = 'l';
                    point2 = 'h';
                }
            }
        }
        
        // find the cutting point N, 
        // along the edge from high to low, 
        // where N.y matches mid.y
        N.set(low.subtract(high));
        N.normalizeLocal();
        float highY = high.getY();
        float midY = mid.getY();
        float Ny = N.add(high).getY(); // low.getY(); // ?
        float dHighMidY = highY - midY;
        float dHighNy = highY - Ny;
        float newMagnitude = dHighMidY / dHighNy;
        N.multLocal(newMagnitude);
        N.addLocal(high);
        
        /* Step 3 - Add the points of the 2 new triangles to array and return */
        
        // With this fourth vertex calculated in the triangle plane,
        // create two new triangles, one with apex up and one with apex down,
        // both sharing the same horizontal cutting line as the base
        // *must determine winding order*

        // HML     or     HLM
        // HMN, LNM    HNM, LMN
        //    H,       H,
        //    /|        |\
        //   / |        | \
        // M/  |N      N|  \M
        //  \``| < or > |``/
        //   \ |        | /
        //    \|        |/
        //    L'       L'
        // Triangle Winding
        
        if(point1 == 'l') {
            if(point2 == 'h') { // LHM 
                splitTris[0] = high.clone();
                splitTris[1] = mid.clone();
                splitTris[2] = N.clone();

                splitTris[3] = low.clone();
                splitTris[4] = N.clone();
                splitTris[5] = mid.clone();
            }else { // point2 == 'm' // LMH 
                splitTris[0] = high.clone();
                splitTris[1] = N.clone();
                splitTris[2] = mid.clone();

                splitTris[3] = low.clone();
                splitTris[4] = mid.clone();
                splitTris[5] = N.clone();
            }
        }else if(point1 == 'm') {
            if(point2 == 'l') { // MLH 
                splitTris[0] = high.clone();
                splitTris[1] = mid.clone();
                splitTris[2] = N.clone();

                splitTris[3] = low.clone();
                splitTris[4] = N.clone();
                splitTris[5] = mid.clone();
            }else { // point2 == 'h' // MHL 
                splitTris[0] = high.clone();
                splitTris[1] = N.clone();
                splitTris[2] = mid.clone();

                splitTris[3] = low.clone();
                splitTris[4] = mid.clone();
                splitTris[5] = N.clone();
            }
        }else { // point1 == 'h'
            if(point2 == 'm') { // HML 
                splitTris[0] = high.clone();
                splitTris[1] = mid.clone();
                splitTris[2] = N.clone();

                splitTris[3] = low.clone();
                splitTris[4] = N.clone();
                splitTris[5] = mid.clone();
            }else { // point2 == 'l' // HLM 
                splitTris[0] = high.clone();
                splitTris[1] = N.clone();
                splitTris[2] = mid.clone();

                splitTris[3] = low.clone();
                splitTris[4] = mid.clone();
                splitTris[5] = N.clone();
            }
        }
        return splitTris;
    }
    
    private Vector3f getPointOfApplicationTop(Vector3f apex, Vector3f baseA, Vector3f baseB) {
        /* Expectation is that verts of tri are in worldSpace 
            apex is up, base is down(XZ plane aligned)
        */
        
        // z0 is apex depth from surface
        // h is height from apex to base
        // tc = length along the median from apex toward midpoint of horizontal base
        //   --formula for center of tri with horizontal base pointing down:
        // tc = (4z0 + 3h) / (6z0 + 4h)
        //   --formula for center of tri with horizontal base pointing up:
        // tc = (2z0 + h) / (6z0 + 2h)
        
        // get midpoint of base to create median with apex
        // From apex =====> toward baseMidpoint
        Vector3f baseMidpoint = baseA.add(baseB).mult(0.5f);
        Vector3f pointAlongMedian = baseMidpoint.clone();
        
        // subtract apex from pointAlongMedian to convert it from position 
        // into vector with length equal to the distance from the apex 
        // to the midpoint of the base (full median)
        pointAlongMedian.subtractLocal(apex);
        
        // get the depth of the apex below the surface (negative if apex is over)
        float apexDepth;
        if(!isGerstnerWave) {
            apexDepth = WaterSurfaceGenerator.getHeightAt(
                            apex.getX(), 
                            apex.getZ(), 
                            waterSurfaceAppState.getElapsedBouyancyTime()) - 
                        apex.getY();
        }else{
            apexDepth = WaterSurfaceGenerator.getGerstnerHeightAt(
                            apex.getX(), 
                            apex.getZ(), 
                            waterSurfaceAppState.getElapsedBouyancyTime()) - 
                        apex.getY();
        }
        
        // get the depth difference from the apex to the base
        float height = apex.getY() - baseMidpoint.getY();
        //System.out.println("Top Triangle | z0: "+ apexDepth +", h: "+ height);
        
        float distFromApex; // (0 to 1)
        // apex up tc = (4z0 + 3h) / (6z0 + 4h)
        distFromApex = (4f*apexDepth + 3f*height) / (6f*apexDepth + 4f*height);
        //System.out.println("Top tc = (4z0 + 3h) / (6z0 + 4h) = "+ distFromApex);
        if(distFromApex > 1f || distFromApex < 0f) { // if not (0 to 1)
            //System.out.println("Top pointAlongMedian distance: "+ distFromApex);
            //System.out.println(" (z0: "+ apexDepth +", h: "+ height);
            //System.out.println(" ("+ (4f*apexDepth) +" + "+ (3f*height) +") / ("+ (6*apexDepth) +" + "+ (4f*height) +")");
            //System.out.println(" pointAlongMedian defaults to Centroid");
            distFromApex = 2f/3f;
        }        
        
        // multiply full median by distFromApex [0 to 1]
        pointAlongMedian.multLocal(distFromApex);
        
        // add the apex back in to return to world space position
        pointAlongMedian.addLocal(apex);
        
        // bring points back into Geometry local space for
        // debug line from apex to point of application 
        Vector3f localApex = new Vector3f();
        Vector3f localPointAlongMedian = new Vector3f();
        principalGeom.worldToLocal(apex, localApex);
        principalGeom.worldToLocal(pointAlongMedian, localPointAlongMedian);
        bouyancyForcePointVertices.add(localApex);
        bouyancyForcePointVertices.add(localPointAlongMedian);
        
        return pointAlongMedian;
    }
    
    private Vector3f getPointOfApplicationBottom(Vector3f apex, Vector3f baseA, Vector3f baseB) {
        /* Expectation is that verts of tri are in worldSpace
            apex is down, base is up(XZ plane aligned) 
        */
        
        // z0 is base depth from surface
        // h is bottomApex depth - base depth
        // tc = length along the median from midpoint of horizontal base toward apex 
        //   --formula for center of tri with horizontal base pointing up:
        // tc = (2z0 + h) / (6z0 + 2h)
        
        // get midpoint of base to create median with apex 
        // From baseMidpoint =====> toward apex
        Vector3f baseMidpoint = baseA.add(baseB).mult(0.5f);
        Vector3f pointAlongMedian = apex.clone();
        
        // subtract baseMidpoint from pointAlongMedian to convert it from position 
        // into vector with length equal to the distance from the baseMidpoint 
        // to the apex (full median)
        pointAlongMedian.subtractLocal(baseMidpoint);
        
        // get the depth of the apex(baseMidpoint in this case) from the surface (negative if apex is higher)
        
        float baseMidpointDepth;
        if(!isGerstnerWave) {
        baseMidpointDepth = WaterSurfaceGenerator.getHeightAt(
                                baseMidpoint.getX(), 
                                baseMidpoint.getZ(), 
                                waterSurfaceAppState.getElapsedBouyancyTime()) -
                            baseMidpoint.getY();
        }else{
            baseMidpointDepth = WaterSurfaceGenerator.getGerstnerHeightAt(
                                baseMidpoint.getX(), 
                                baseMidpoint.getZ(), 
                                waterSurfaceAppState.getElapsedBouyancyTime()) -
                            baseMidpoint.getY();
        }
        
        // get the vertical depth from the apex height to the baseMidpoint height
        float height =  baseMidpoint.getY() - apex.getY();
        //System.out.println("Bottom Triangle | z0: "+ baseMidpointDepth +", h: "+ height);
        
        float distFromBaseMidpoint; // (0 to 1)
        // downApex tc = (2z0 + h) / (6z0 + 2h)
        distFromBaseMidpoint = (2f*baseMidpointDepth + height) / (6f*baseMidpointDepth + 2f*height);
        //System.out.println("Bottom tc = (2z0 + h) / (6z0 + 2h) = "+ distFromBaseMidpoint);
        if(distFromBaseMidpoint > 1f || distFromBaseMidpoint < 0f) {
            //System.out.println("Bottom pointAlongMedian distance: "+ distFromBaseMidpoint);
            //System.out.println(" (z0: "+ baseMidpointDepth +" h: "+ height);
            //System.out.println(" ("+ (2f*baseMidpointDepth) +" + "+ height +") / ("+ (6f*baseMidpointDepth) +" + "+ (2f*height) +")");
            //System.out.println(" pointAlongMedian defaults to Centroid");
            distFromBaseMidpoint = 1f/3f;
        }
        
        // multiply full median by distFromBaseMidpoint [0 to 1]
        pointAlongMedian.multLocal(distFromBaseMidpoint);
        
        // add the baseMidpoint back in to return to world space
        pointAlongMedian.addLocal(baseMidpoint);
        
        // bring points back into Geometry local space for
        // debug line from apex to point of application 
        Vector3f localBaseMidpoint = new Vector3f();
        Vector3f localPointAlongMedian = new Vector3f();
        principalGeom.worldToLocal(baseMidpoint, localBaseMidpoint);
        principalGeom.worldToLocal(pointAlongMedian, localPointAlongMedian);
        bouyancyForcePointVertices.add(localBaseMidpoint);
        bouyancyForcePointVertices.add(localPointAlongMedian);
        
        return pointAlongMedian;
    }
    
    private void zeroVectorArray(Vector3f[] array) {
        for(int i=0; i < array.length; i++) {
            array[i] = new Vector3f(0f,0f,0f);
        }
    }
    
    private float clampedGAMMA(float GAMMA, float maxGAMMA) {
        float result = 0f;
        if(GAMMA > 0f && maxGAMMA > 0f) {
            result = GAMMA / maxGAMMA;
            if(result > 1f) {
                result = 1f;
            }
        }
        return result;
    }
    
    private boolean isValidForce(Vector3f force) {
        //return !Float.isNaN(force.getX() + force.getY() + force.getZ());
        return !(Float.isNaN(force.getX())
                || Float.isNaN(force.getY())
                || Float.isNaN(force.getZ()));
    }
    
    private void applyDamping(boolean quadratic) {
        float dampingCoefficient = 8f;// 15?
        float submergedSurfaceRatio = totalSubmergedSurfaceArea / totalSurfaceArea;
        Vector3f currentVelocity = rigidBodyControl.getLinearVelocity();
        Vector3f currentAngularVelocity = rigidBodyControl.getAngularVelocity();
        float speedSquared = currentVelocity.lengthSquared();
        Vector3f dampingForce;
        Vector3f dampingTorque;
        
        if(quadratic){
            dampingForce = currentVelocity.normalize().
                                mult(dampingCoefficient).
                                    mult(submergedSurfaceRatio).
                                        mult(speedSquared).negate();
            dampingTorque = currentAngularVelocity.normalize().
                                mult(dampingCoefficient).
                                    mult(submergedSurfaceRatio).
                                        mult(speedSquared).negate();
            
        }else{ // Linear
            dampingForce = currentVelocity.
                            mult(dampingCoefficient).
                                mult(submergedSurfaceRatio).negate();
            dampingTorque = currentAngularVelocity.
                            mult(dampingCoefficient).
                                mult(submergedSurfaceRatio).negate();
        }
        
        rigidBodyControl.applyImpulse(dampingForce, new Vector3f(0,0,0));
        rigidBodyControl.applyTorqueImpulse(dampingTorque);
    }
    
    
    // DEBUG
    
    private void initDebugWaterLine() {
        // debug Mesh for the waterline (Mesh.Mode.Lines)
        waterLineMesh = new Mesh();
        waterLineMesh.setDynamic();
        waterLineMesh.setMode(Mesh.Mode.Lines);
        Material waterLineMat = new Material(waterSurfaceAppState.getApp().getAssetManager(),
                                                "Common/MatDefs/Misc/Unshaded.j3md");
        waterLineMat.setColor("Color", new ColorRGBA(0.4f, 0.5f, 0.9f, 1f));
        //waterLineMat.getAdditionalRenderState().setFaceCullMode(FaceCullMode.Back); // doesnt work with Mesh.Mode.Lines (no normals)
        //waterLineMat.getAdditionalRenderState().setWireframe(true); // not needed with Mesh.Mode.Lines?
        waterLineMat.getAdditionalRenderState().setLineWidth(3f);
        waterLineGeom = new Geometry("Water Line Geometry", waterLineMesh);
        waterLineGeom.setMaterial(waterLineMat);
        //waterLineGeom.setCullHint(Spatial.CullHint.Never); // set to Always to make invisible
        principalGeom.getParent().attachChild(waterLineGeom);
        //waterLineGeom.setLocalTranslation(principalGeom.getLocalTranslation());
        waterLineGeom.scale(1.001f);
    }
    
    private void updateDebugWaterLine() {
        // create waterline Mesh (Mesh.Mode.Lines)
        Vector3f[] waterLineVertices = new Vector3f[0];
        waterLineVertices = waterLineVerts.toArray(waterLineVertices);
        waterLineMesh.setBuffer(VertexBuffer.Type.Position, 3, BufferUtils.createFloatBuffer(waterLineVertices));
        waterLineMesh.updateCounts();
        waterLineMesh.updateBound();
        //waterLineGeom.updateModelBound(); // not needed?
    }
    
    private void initDebugSubmergedMesh() {
        // debug Mesh showing the submerged triangles of the original Mesh
        submergedMesh = new Mesh();
        submergedMesh.setDynamic();
        Material submergedMatUnlit = new Material(waterSurfaceAppState.getApp().getAssetManager(),
                                                "Common/MatDefs/Misc/Unshaded.j3md");
        submergedMatUnlit.setColor("Color", ColorRGBA.Magenta);
        submergedMatUnlit.getAdditionalRenderState().setWireframe(true);
        submergedMatUnlit.getAdditionalRenderState().setLineWidth(2f);
        //submergedMatUnlit.getAdditionalRenderState().setFaceCullMode(FaceCullMode.Off);
        
        Material submergedMat = new Material(waterSurfaceAppState.getApp().getAssetManager(),
                                    "Common/MatDefs/Light/Lighting.j3md");
        submergedMat.setBoolean("UseMaterialColors",true);
        submergedMat.setColor("Diffuse", new ColorRGBA(0.9f, 0.5f, 1f, 1f)); //ColorRGBA.Red);
        submergedMat.setColor("Ambient", new ColorRGBA(0.9f, 0.5f, 1f, 1f));
        submergedMat.setColor("Specular", ColorRGBA.White);
        submergedMat.setFloat("Shininess", 0f);  // [0,128]
        submergedMat.getAdditionalRenderState().setFaceCullMode(FaceCullMode.Off);
        
        submergedGeom = new Geometry("Submerged Geometry", submergedMesh);
        submergedGeom.setMaterial(submergedMat);
        //submergedGeom.scale(1.0001f);
        submergedGeom.scale(0.9999f);
        //submergedGeom.setCullHint(Spatial.CullHint.Never);
        principalNode.attachChild(submergedGeom);
        //submergedGeom.setLocalTranslation(principalGeom.getLocalTranslation());
        
    }
    
    private void updateDebugSubmergedMesh() {
        // create position array
        Vector3f[] submergedVerticesArray = new Vector3f[0];
        submergedVerticesArray = submergedVertices.toArray(submergedVerticesArray);
        // create index array
        int[] submergedIndicesArray = new int[submergedIndices.size()];
        for(int count =0; count < submergedIndices.size(); count++){
            submergedIndicesArray[count] = submergedIndices.get(count);
        }
        // create normals array (should these be in local space?)
        Vector3f[] submergedNormalsArray; // = new Vector3f[0];
        submergedNormalsArray = getCurrentMeshWorldNormals(submergedVerticesArray, submergedIndicesArray);
        
        // set Mesh position and index buffers and update Mesh
        submergedMesh.setBuffer(VertexBuffer.Type.Position, 3, BufferUtils.createFloatBuffer(submergedVerticesArray));
        submergedMesh.setBuffer(VertexBuffer.Type.Normal, 3, BufferUtils.createFloatBuffer(submergedNormalsArray));
        submergedMesh.setBuffer(VertexBuffer.Type.Index, 1, BufferUtils.createIntBuffer(submergedIndicesArray));
        submergedMesh.updateCounts();
        submergedMesh.updateBound();
    }
    
    private void initDebugSplitSubMesh() {
        // debug Mesh showing the submerged triangles of the original Mesh
        splitSubMesh = new Mesh();
        splitSubMesh.setDynamic();
        Material splitSubMatUnlit = new Material(waterSurfaceAppState.getApp().getAssetManager(),
                                                "Common/MatDefs/Misc/Unshaded.j3md");
        splitSubMatUnlit.setColor("Color", ColorRGBA.Magenta);
        splitSubMatUnlit.getAdditionalRenderState().setWireframe(true);
        splitSubMatUnlit.getAdditionalRenderState().setLineWidth(2f);
        splitSubMatUnlit.getAdditionalRenderState().setFaceCullMode(FaceCullMode.Off);
        
        Material splitSubMat = new Material(waterSurfaceAppState.getApp().getAssetManager(),
                                    "Common/MatDefs/Light/Lighting.j3md");
        splitSubMat.setBoolean("UseMaterialColors",true);
        splitSubMat.setColor("Diffuse", ColorRGBA.Magenta);
        splitSubMat.setColor("Ambient",ColorRGBA.White);
        splitSubMat.setColor("Specular",ColorRGBA.White);
        splitSubMat.setFloat("Shininess", 0f);  // [0,128]
        //splitSubMat.getAdditionalRenderState().setFaceCullMode(FaceCullMode.Off);
        
        splitSubGeom = new Geometry("Split Submerged Geometry", splitSubMesh);
        splitSubGeom.setMaterial(splitSubMatUnlit);
        //splitSubGeom.setCullHint(Spatial.CullHint.Never);
        
        principalGeom.getParent().attachChild(splitSubGeom);
        //splitSubGeom.setLocalTranslation(principalGeom.getLocalTranslation());
    }
    
    private void updateDebugSplitSubMesh() {
        // create position array
        Vector3f[] splitSubVerticesArray = new Vector3f[0];
        splitSubVerticesArray = splitSubVertices.toArray(splitSubVerticesArray);
        // create index array
        int[] splitSubIndicesArray = new int[splitSubIndices.size()];
        for(int count =0; count < splitSubIndices.size(); count++){
            splitSubIndicesArray[count] = splitSubIndices.get(count);
        }
        // create normals array
        Vector3f[] splitSubNormalsArray = new Vector3f[0];
        splitSubNormalsArray = getCurrentMeshWorldNormals(splitSubVerticesArray, splitSubIndicesArray);
        
        // set Mesh position and index buffers and update Mesh
        splitSubMesh.setBuffer(VertexBuffer.Type.Position, 3, BufferUtils.createFloatBuffer(splitSubVerticesArray));
        splitSubMesh.setBuffer(VertexBuffer.Type.Normal, 3, BufferUtils.createFloatBuffer(splitSubNormalsArray));
        splitSubMesh.setBuffer(VertexBuffer.Type.Index, 1, BufferUtils.createIntBuffer(splitSubIndicesArray));
        splitSubMesh.updateCounts();
        splitSubMesh.updateBound();
    }
    
    private void initDebugSplitSubRejectedMesh() {
        // debug Mesh showing the Rejected submerged triangles of the original Mesh
        splitSubRejectedMesh = new Mesh();
        splitSubRejectedMesh.setDynamic();
        Material splitSubRejectedMatUnlit = new Material(waterSurfaceAppState.getApp().getAssetManager(),
                                                "Common/MatDefs/Misc/Unshaded.j3md");
        splitSubRejectedMatUnlit.setColor("Color", ColorRGBA.Yellow);
        splitSubRejectedMatUnlit.getAdditionalRenderState().setWireframe(true);
        splitSubRejectedMatUnlit.getAdditionalRenderState().setLineWidth(2f);
        splitSubRejectedMatUnlit.getAdditionalRenderState().setFaceCullMode(FaceCullMode.Off);
        
        Material splitSubRejectedMat = new Material(waterSurfaceAppState.getApp().getAssetManager(),
                                    "Common/MatDefs/Light/Lighting.j3md");
        splitSubRejectedMat.setBoolean("UseMaterialColors",true);
        splitSubRejectedMat.setColor("Diffuse", ColorRGBA.Yellow);
        splitSubRejectedMat.setColor("Ambient",ColorRGBA.White);
        splitSubRejectedMat.setColor("Specular",ColorRGBA.White);
        splitSubRejectedMat.setFloat("Shininess", 0f);  // [0,128]
        //splitSubRejectedMat.getAdditionalRenderState().setFaceCullMode(FaceCullMode.Off);
        
        splitSubRejectedGeom = new Geometry("Split Submerged Rejected Geometry", splitSubRejectedMesh);
        splitSubRejectedGeom.setMaterial(splitSubRejectedMatUnlit);
        //splitSubRejectedGeom.setCullHint(Spatial.CullHint.Never);
        
        principalGeom.getParent().attachChild(splitSubRejectedGeom);
        //splitSubRejectedGeom.setLocalTranslation(principalGeom.getLocalTranslation());
    }
    
    private void updateDebugSplitSubRejectedMesh() {
        // create position array
        Vector3f[] splitSubRejectedVerticesArray = new Vector3f[0];
        splitSubRejectedVerticesArray = splitSubRejectedVertices.toArray(splitSubRejectedVerticesArray);
        // create index array
        int[] splitSubRejectedIndicesArray = new int[splitSubRejectedIndices.size()];
        for(int count =0; count < splitSubRejectedIndices.size(); count++){
            splitSubRejectedIndicesArray[count] = splitSubRejectedIndices.get(count);
        }
        // create normals array
        Vector3f[] splitSubRejectedNormalsArray; // = new Vector3f[0];
        splitSubRejectedNormalsArray = getCurrentMeshWorldNormals(splitSubRejectedVerticesArray, splitSubRejectedIndicesArray);
        
        // set Mesh position and index buffers and update Mesh
        splitSubRejectedMesh.setBuffer(VertexBuffer.Type.Position, 3, BufferUtils.createFloatBuffer(splitSubRejectedVerticesArray));
        splitSubRejectedMesh.setBuffer(VertexBuffer.Type.Normal, 3, BufferUtils.createFloatBuffer(splitSubRejectedNormalsArray));
        splitSubRejectedMesh.setBuffer(VertexBuffer.Type.Index, 1, BufferUtils.createIntBuffer(splitSubRejectedIndicesArray));
        splitSubRejectedMesh.updateCounts();
        splitSubRejectedMesh.updateBound();
    }
    
    private void initDebugSplitSubNormalsMesh() {
        // debug Line Mesh showing the normals of current split submerged triangles
        splitSubNormalsMesh = new Mesh();
        splitSubNormalsMesh.setDynamic();
        splitSubNormalsMesh.setMode(Mesh.Mode.Lines); //Mesh.Mode.Points
        Material  splitSubNormalsMatUnlit = new Material(waterSurfaceAppState.getApp().getAssetManager(),
                                                "Common/MatDefs/Misc/Unshaded.j3md");
         splitSubNormalsMatUnlit.setColor("Color", ColorRGBA.Blue);
        // splitSubNormalsMatUnlit.getAdditionalRenderState().setWireframe(true);
         splitSubNormalsMatUnlit.getAdditionalRenderState().setLineWidth(1f);
        // splitSubNormalsMatUnlit.getAdditionalRenderState().setFaceCullMode(FaceCullMode.Off);
        
        splitSubNormalsGeom = new Geometry("Normals Geometry",  splitSubNormalsMesh);
        splitSubNormalsGeom.setMaterial( splitSubNormalsMatUnlit);
        // splitSubNormalsGeom.setCullHint(Spatial.CullHint.Never);
        
        principalGeom.getParent().attachChild( splitSubNormalsGeom);
    }
    
    private void updateDebugSplitSubNormalsMesh() {
        // create position array
        Vector3f[] splitSubNormalsVerticesArray = new Vector3f[0];
        splitSubNormalsVerticesArray = splitSubNormalsVertices.toArray(splitSubNormalsVerticesArray);
        
        // set Mesh vertex positions and update Mesh
        splitSubNormalsMesh.setBuffer(VertexBuffer.Type.Position, 3, BufferUtils.createFloatBuffer(splitSubNormalsVerticesArray));
        splitSubNormalsMesh.updateCounts();
        splitSubNormalsMesh.updateBound();
    }
    
    private void initDebugForcePointMesh() {
        // debug Line Mesh showing the points of force application on current split submerged triangles
        forcePointMesh = new Mesh();
        forcePointMesh.setDynamic();
        forcePointMesh.setMode(Mesh.Mode.Lines);
        Material forcePointMatUnlit = new Material(waterSurfaceAppState.getApp().getAssetManager(),
                                                "Common/MatDefs/Misc/Unshaded.j3md");
        forcePointMatUnlit.setColor("Color", ColorRGBA.Orange);
        //forcePointMatUnlit.getAdditionalRenderState().setWireframe(true);
        forcePointMatUnlit.getAdditionalRenderState().setLineWidth(2f);
        //forcePointMatUnlit.getAdditionalRenderState().setFaceCullMode(FaceCullMode.Off);
        
        forcePointGeom = new Geometry("Force Point Geometry", forcePointMesh);
        forcePointGeom.setMaterial(forcePointMatUnlit);
        //forcePointGeom.setCullHint(Spatial.CullHint.Never);
        
        principalGeom.getParent().attachChild(forcePointGeom);
    }
    
    private void updateDebugForcePointMesh() {
        // create position array
        Vector3f[] forcePointVerticesArray = new Vector3f[0];
        forcePointVerticesArray = bouyancyForcePointVertices.toArray(forcePointVerticesArray);
        
        // set Mesh vertex positions and update Mesh
        forcePointMesh.setBuffer(VertexBuffer.Type.Position, 3, BufferUtils.createFloatBuffer(forcePointVerticesArray));
        //forcePointMesh.setBuffer(VertexBuffer.Type.Index, 1, BufferUtils.createIntBuffer(forcePointIndicesArray));
        forcePointMesh.updateCounts();
        forcePointMesh.updateBound();
    }
    
    private void initDebugBouyancyForceVectorMesh() {
        // debug Line Mesh showing the current bouyancy force applied to each submerged triangle
        forceVectorMesh = new Mesh();
        forceVectorMesh.setDynamic();
        forceVectorMesh.setMode(Mesh.Mode.Lines);
        Material forceVectorMatUnlit = new Material(waterSurfaceAppState.getApp().getAssetManager(),
                                                "Common/MatDefs/Misc/Unshaded.j3md");
        forceVectorMatUnlit.setColor("Color", ColorRGBA.Cyan);
        //forceVectorMatUnlit.getAdditionalRenderState().setWireframe(true);
        forceVectorMatUnlit.getAdditionalRenderState().setLineWidth(1f);
        //forceVectorMatUnlit.getAdditionalRenderState().setFaceCullMode(FaceCullMode.Off);
        
        forceVectorGeom = new Geometry("Force Vector Geometry", forceVectorMesh);
        forceVectorGeom.setMaterial(forceVectorMatUnlit);
        //forceVectorGeom.setCullHint(Spatial.CullHint.Never);
        
        principalGeom.getParent().attachChild(forceVectorGeom);
    }
    
    private void updateDebugBouyancyForceVectorMesh() {
        // create position array
        Vector3f[] forceVectorVerticesArray = new Vector3f[0];
        forceVectorVerticesArray = bouyancyForceVectorVertices.toArray(forceVectorVerticesArray);
        
        // set Mesh vertex positions and update Mesh
        forceVectorMesh.setBuffer(VertexBuffer.Type.Position, 3, BufferUtils.createFloatBuffer(forceVectorVerticesArray));
        forceVectorMesh.updateCounts();
        forceVectorMesh.updateBound();
    }
    
    private void initDebugSubmergedTriangleVelocityMesh() {
        // debug Line Mesh showing the velocities of all submerged triangles
        submergedTriangleVelocityMesh = new Mesh();
        submergedTriangleVelocityMesh.setDynamic();
        submergedTriangleVelocityMesh.setMode(Mesh.Mode.Lines);
        Material submergedTriangleVelocityMatUnlit = new Material(waterSurfaceAppState.getApp().getAssetManager(),
                                                "Common/MatDefs/Misc/Unshaded.j3md");
        submergedTriangleVelocityMatUnlit.setColor("Color", ColorRGBA.Pink);
        submergedTriangleVelocityMatUnlit.getAdditionalRenderState().setLineWidth(1f);
        //submergedTriangleVelocityMatUnlit.getAdditionalRenderState().setWireframe(true);
        //submergedTriangleVelocityMatUnlit.getAdditionalRenderState().setFaceCullMode(FaceCullMode.Off);
        
        submergedTriangleVelocityGeom = new Geometry("Submerged Triangle Velocity Vector Geometry", submergedTriangleVelocityMesh);
        submergedTriangleVelocityGeom.setMaterial(submergedTriangleVelocityMatUnlit);
        //submergedTriangleVelocityGeom.setCullHint(Spatial.CullHint.Never);
        
        principalGeom.getParent().attachChild(submergedTriangleVelocityGeom);
    }
       
    private void updateDebugSubmergedTriangleVelocityMesh() {
        // create position array
        Vector3f[] submergedTriangleVelocityVerticesArray = new Vector3f[0];
        submergedTriangleVelocityVerticesArray = submergedTriangleVelocityVertices.toArray(submergedTriangleVelocityVerticesArray);
        
        // set Mesh vertex positions and update Mesh
        submergedTriangleVelocityMesh.setBuffer(VertexBuffer.Type.Position, 3, BufferUtils.createFloatBuffer(submergedTriangleVelocityVerticesArray));
        submergedTriangleVelocityMesh.updateCounts();
        submergedTriangleVelocityMesh.updateBound();
    }
    
    private void initDebugPrincipalTriangleVelocityMesh() {
        // debug Line Mesh showing the velocities of all submerged triangles
        principalTriangleVelocityMesh = new Mesh();
        principalTriangleVelocityMesh.setDynamic();
        principalTriangleVelocityMesh.setMode(Mesh.Mode.Lines);
        Material principalTriangleVelocityMatUnlit = new Material(waterSurfaceAppState.getApp().getAssetManager(),
                                                "Common/MatDefs/Misc/Unshaded.j3md");
        principalTriangleVelocityMatUnlit.setColor("Color", ColorRGBA.Pink);
        principalTriangleVelocityMatUnlit.getAdditionalRenderState().setLineWidth(1f);
        //principalTriangleVelocityMatUnlit.getAdditionalRenderState().setWireframe(true);
        //principalTriangleVelocityMatUnlit.getAdditionalRenderState().setFaceCullMode(FaceCullMode.Off);
        
        principalTriangleVelocityGeom = new Geometry("Principal Triangle Velocity Vector Geometry", principalTriangleVelocityMesh);
        principalTriangleVelocityGeom.setMaterial(principalTriangleVelocityMatUnlit);
        //principalTriangleVelocityGeom.setCullHint(Spatial.CullHint.Never);
        
        principalGeom.getParent().attachChild(principalTriangleVelocityGeom);
    }
    
    private void updateDebugPrincipalTriangleVelocityMesh() {
        // create position array
        Vector3f[] principalTriangleVelocityVerticesArray = new Vector3f[0];
        principalTriangleVelocityVerticesArray = principalTriangleVelocityVertices.toArray(principalTriangleVelocityVerticesArray);
        
        // set Mesh vertex positions and update Mesh
        principalTriangleVelocityMesh.setBuffer(VertexBuffer.Type.Position, 3, BufferUtils.createFloatBuffer(principalTriangleVelocityVerticesArray));
        principalTriangleVelocityMesh.updateCounts();
        principalTriangleVelocityMesh.updateBound();
    }
    
    private void initDebugResistanceForceVectorMesh() {
        // debug Line Mesh showing the force vectors applied to all submerged triangles
        resistanceForceVectorMesh = new Mesh();
        resistanceForceVectorMesh.setDynamic();
        resistanceForceVectorMesh.setMode(Mesh.Mode.Lines); //Mesh.Mode.Points
        Material resistanceForceVectorMatUnlit = new Material(waterSurfaceAppState.getApp().getAssetManager(),
                                                "Common/MatDefs/Misc/Unshaded.j3md");
        resistanceForceVectorMatUnlit.setColor("Color", ColorRGBA.Yellow);
        resistanceForceVectorMatUnlit.getAdditionalRenderState().setLineWidth(1f);
        //resistanceForceVectorMatUnlit.getAdditionalRenderState().setWireframe(true);
        //resistanceForceVectorMatUnlit.getAdditionalRenderState().setFaceCullMode(FaceCullMode.Off);
        
        resistanceForceVectorGeom = new Geometry("Resistance Force Vector Geometry", resistanceForceVectorMesh);
        resistanceForceVectorGeom.setMaterial(resistanceForceVectorMatUnlit);
        //resistanceForceVectorGeom.setCullHint(Spatial.CullHint.Never);
        
        principalGeom.getParent().attachChild(resistanceForceVectorGeom);
    }
    
    private void updateDebugResistanceForceVectorMesh() {
        // create position array
        Vector3f[] resistanceForceVectorVerticesArray = new Vector3f[0];
        resistanceForceVectorVerticesArray = resistanceForceVectorVertices.toArray(resistanceForceVectorVerticesArray);
        
        // set Mesh vertex positions and update Mesh
        resistanceForceVectorMesh.setBuffer(VertexBuffer.Type.Position, 3, BufferUtils.createFloatBuffer(resistanceForceVectorVerticesArray));
        //resistanceForceVectorMesh.setBuffer(VertexBuffer.Type.Index, 1, BufferUtils.createIntBuffer(resistanceForceVectorIndicesArray));
        resistanceForceVectorMesh.updateCounts();
        resistanceForceVectorMesh.updateBound();
    }
    
    private void initDebugSlammingForceVectorMesh() {
        // debug Line Mesh showing the slamming force applied to each principal triangle
        slammingForceVectorMesh = new Mesh();
        slammingForceVectorMesh.setDynamic();
        slammingForceVectorMesh.setMode(Mesh.Mode.Lines);
        Material slammingForceVectorMatUnlit = new Material(waterSurfaceAppState.getApp().getAssetManager(),
                                                "Common/MatDefs/Misc/Unshaded.j3md");
        slammingForceVectorMatUnlit.setColor("Color", ColorRGBA.Red);
        slammingForceVectorMatUnlit.getAdditionalRenderState().setLineWidth(1f);
        //slammingForceVectorMatUnlit.getAdditionalRenderState().setWireframe(true);
        //slammingForceVectorMatUnlit.getAdditionalRenderState().setFaceCullMode(FaceCullMode.Off);
        
        slammingForceVectorGeom = new Geometry("Slamming Force Vector Geometry", slammingForceVectorMesh);
        slammingForceVectorGeom.setMaterial(slammingForceVectorMatUnlit);
        //slammingForceVectorGeom.setCullHint(Spatial.CullHint.Never);
        
        principalGeom.getParent().attachChild(slammingForceVectorGeom);
    }
    
    private void updateDebugSlammingForceVectorMesh() {
        // create position array
        Vector3f[] slammingForceVectorVerticesArray = new Vector3f[0];
        slammingForceVectorVerticesArray = slammingForceVectorVertices.toArray(slammingForceVectorVerticesArray);
        
        // set Mesh vertex positions and update Mesh
        slammingForceVectorMesh.setBuffer(VertexBuffer.Type.Position, 3, BufferUtils.createFloatBuffer(slammingForceVectorVerticesArray));
        slammingForceVectorMesh.updateCounts();
        slammingForceVectorMesh.updateBound();
    }
    
    // END DEBUG
    
    
    private void addToSplitSubMesh(Vector3f p1, Vector3f p2, Vector3f p3) {
        splitSubVertices.add(p1.clone());
        splitSubVertices.add(p2.clone());
        splitSubVertices.add(p3.clone());
        splitSubIndices.add(splitSubIndices.size());
        splitSubIndices.add(splitSubIndices.size());
        splitSubIndices.add(splitSubIndices.size());
    }
    
    private void addToSplitSubRejectedMesh(Vector3f p1, Vector3f p2, Vector3f p3) {
        splitSubRejectedVertices.add(p1.clone());
        splitSubRejectedVertices.add(p2.clone());
        splitSubRejectedVertices.add(p3.clone());
        splitSubRejectedIndices.add(splitSubRejectedIndices.size());
        splitSubRejectedIndices.add(splitSubRejectedIndices.size());
        splitSubRejectedIndices.add(splitSubRejectedIndices.size());
    }
    
    private Vector3f[] getCurrentMeshWorldNormals(Vector3f[] vertices, int[] indices) {
        if(vertices.length < 3 || indices.length < 3) {
            return new Vector3f[0];
        }
        Vector3f[] normals = new Vector3f[vertices.length];
        Vector3f normalVector;
        Vector3f worldP1 = new Vector3f();
        Vector3f worldP2 = new Vector3f();
        Vector3f worldP3 = new Vector3f();
        for(int i = 0; i < normals.length; i += 3) {
              // transform the local triangle verts to their world translations
            worldP1 = principalGeom.localToWorld(vertices[indices[i]], worldP1);
            worldP2 = principalGeom.localToWorld(vertices[indices[i+1]], worldP2);
            worldP3 = principalGeom.localToWorld(vertices[indices[i+2]], worldP3);
              // get the normal of the triangle in world space
            normalVector = getTriangleNormal(worldP1, worldP2, worldP3);
              // share normal for all 3 verts that make up triangle
            normals[i] = normalVector;
            normals[i+1] = normalVector;
            normals[i+2] = normalVector;
        }
        return normals;
    }
    
    private Vector3f getTriangleNormal(Vector3f p1, Vector3f p2, Vector3f p3) {
        return (p2.subtract(p1).cross(p3.subtract(p1))).normalize();
    }
    
    private float getTriangleArea(Vector3f p1, Vector3f p2, Vector3f p3) {
        // Heron's Formula
        // A, B, C = lengths of the 3 sides
        // S = (A+B+C)/2 = length of the SemiPerimeter 
        // Area = sqrt(S*(S-A)*(S-B)*(S-C))
        float A = p2.subtract(p1).length();
        float B = p3.subtract(p2).length();
        float C = p1.subtract(p3).length();
        float S = (A + B + C)/2;
        return FastMath.sqrt(S * (S-A) * (S-B) * (S-C));
        
    }
    
    private void setupPrincipalSurfaceAreas(Vector3f[] vertices, int[] indices) {
        int triangleCounter = 0;
        float currentArea;
        for(int i = 0; i < indices.length; i +=3) {
            currentArea = getTriangleArea(  vertices[indices[i]],
                                            vertices[indices[i+1]],
                                            vertices[indices[i+2]] );
            principalAreas[triangleCounter] = currentArea;
            totalSurfaceArea += currentArea;
            triangleCounter++;            
        }
    }
    
    private boolean isHorizontal(Triangle tri) {
        //return tri.get1().getY() == tri.get2().getY() && tri.get2().getY() == tri.get3().getY();
        return (tri.get1().getY() - tri.get2().getY() < areaThreshold 
                && tri.get2().getY() - tri.get3().getY() < areaThreshold);
    }
    
    private boolean isHorizontal(Vector3f p1, Vector3f p2, Vector3f p3) {
        return (p1.getY() - p2.getY() < areaThreshold 
                && p2.getY() - p3.getY() < areaThreshold);
    }
    
    private Vector3f getTriangleCenter(Vector3f p1, Vector3f p2, Vector3f p3) {
        return p1.add(p2).add(p3).divide(3);
    }
    
    private void setupChaseCam() {
        SimpleApplication app = waterSurfaceAppState.getApp();
        
        // Disable the default flyby cam
        app.getFlyByCamera().setEnabled(false);
 
//        // ** Camera Node **
//        //create the camera Node
//        CameraNode camNode = new CameraNode("Camera Node", app.getCamera());
//        
//        //This mode means that camera copies the movements of the target:
//        camNode.setControlDir(ControlDirection.SpatialToCamera);
//        
//        //Attach the camNode to the target:
//        principalNode.attachChild(camNode);
//        
//        //Move camNode, e.g. behind and above the target:
//        camNode.setLocalTranslation(new Vector3f(0, 7, -20));
//        
//        //Rotate the camNode to look at the target:
//        camNode.lookAt(principalNode.getLocalTranslation(), Vector3f.UNIT_Y);

        // ** Chase Cam **
        ChaseCamera chaseCam = new ChaseCamera(app.getCamera(), principalNode, app.getInputManager());
        chaseCam.setSmoothMotion(true);
        chaseCam.setTrailingEnabled(true);
        chaseCam.setLookAtOffset(new Vector3f(0f, 2f, 5f));
                
    }
    
    
    
    
    public void debugReset() {
        waterSurfaceAppState.getApp().getStateManager().getState(
                SimpleBoatAppState.class).initializeOrientation();
        rigidBodyControl.setLinearVelocity(new Vector3f(0f,0f,0f));
        rigidBodyControl.setAngularVelocity(new Vector3f(0f,0f,0f));
        rigidBodyControl.clearForces(); // broken
        
    }
    
    public boolean isActive() {
        return isActive;
    }
    
    public void setActive(boolean isActive) {
        this.isActive = isActive;
        if(isActive == false) {
            linearVelocityStore = rigidBodyControl.getLinearVelocity().clone();
            angularVelocityStore = rigidBodyControl.getAngularVelocity().clone();
            rigidBodyControl.setAngularVelocity(new Vector3f(0f,0f,0f));
            rigidBodyControl.setLinearVelocity(new Vector3f(0f,0f,0f));
            rigidBodyControl.setGravity(new Vector3f(0f,0f,0f));
        }else{
            rigidBodyControl.setAngularVelocity(angularVelocityStore);
            rigidBodyControl.setLinearVelocity(linearVelocityStore);
            rigidBodyControl.setGravity(gravity);
        }
    }
    
    public boolean isMovingForward() {
        return movingForward;
    }
    
    public void setMovingForward(boolean movingForward) {
        this.movingForward = movingForward;
    }
    
    public boolean isMovingBackward() {
        return movingBackward;
    }
    
    public void setMovingBackward(boolean movingBackward) {
        this.movingBackward = movingBackward;
    }
    
    public boolean isTurningLeft() {
        return turningLeft;
    }
    
    public void setTurningLeft(boolean turningLeft) {
        this.turningLeft = turningLeft;
    }
    
    public boolean isTurningRight() {
        return turningRight;
    }
    
    public void setTurningRight(boolean turningRight) {
        this.turningRight = turningRight;
    }
    
    public float getFluidDensity() {
        return fluidDensity;
    }

    public void setFluidDensity(float fluidDensity) {
        this.fluidDensity = fluidDensity;
    }
    
    
    
    

}// End of Class
