package bouyancy.surface;

import bouyancy.debug.Utils;
import com.jme3.math.FastMath;
import com.jme3.math.Vector2f;
import com.jme3.math.Vector3f;
import java.util.Arrays;

/**
 *
 * @author User (taken from https://developer.nvidia.com/gpugems/GPUGems/gpugems_ch01.html)
 */
public class WaterSurfaceGenerator {
    private static int waveLayers = 0;
    private static float chop = 2.4f; // 1.0f to 2.5f
    private static boolean isChoppy = false;
    
    
    private static WaveProperties[] waves = null;
    // float wavelength, float amplitude, float speed, Vector2f direction-/-centerPosition
       
//    static {
//        addWave(new WaveProperties( 1000f, 0.0001f, 1f, new Vector2f(0.5f, 0.5f) ) );
//    }
    
//    static { // keep the wave directions roughly the same 
//        //addWave(new WaveProperties( 100f, 0.25f, 2f, new Vector2f(0.89590293f, -0.4442499f)));
//        addWave(new WaveProperties( 400f, 1.2f, 1f, new Vector2f(-1, -1) ));// big
//        addWave(new WaveProperties( 100f, 0.2f, 2f, new Vector2f(0, -1) ));
//        addWave(new WaveProperties( 90f, 0.2f, 2f, new Vector2f(-0.4442499f, -0.89590293f) ));
//        addWave(new WaveProperties( 80f, 0.2f, 2f, new Vector2f(0.4442499f, -0.89590293f) ));
//        //addWave(new WaveProperties( 50f, 0.125f, 2f, new Vector2f(10, 10), true )); // circular
//    }
    
//    static { // circular ripples, direction becomes center position
//       addWave(new WaveProperties( 10f, 0.125f, 1f, new Vector2f(0f, 0f), true ));
//    }
    
//    static { // big rolling waves
//        addWave(new WaveProperties( 100f, 0.5f, 2f, new Vector2f(0.89590293f, -0.4442499f) ));
//    }
    
//    static { // natural ?
//        addWave(new WaveProperties( 10.700179f, 0.08987389f, 2.334795f, new Vector2f(0.89590293f, -0.4442499f) ));
//    }
    
//    static {
//        addWave(Utils.getRandomWaveConstrained(new Vector2f(0.7f, 0.3f).normalize()));
//    }
    
//    static {
//        addWave(new WaveProperties(100f, 0.5f, 2f, Utils.randomDirection2d()));
//    }
    
//    // print out initial collection of wave properties
//    static {
//        for(WaveProperties wave : waves) {
//            System.out.println(wave.getWavelength()+", "+
//                                wave.getAmplitude()+" ,"+
//                                wave.getSpeed()+", "+
//                                wave.getDirection());
//        }
//    }
    
     
    public static int getWaveLayers() {
        return waveLayers;
    }
    
    // clears waveLayer count and re-initializes empty waves array
    public static void clearWaves() {
        waves = new WaveProperties[0];
        waveLayers = 0;
    }
   
    public static void addWave(WaveProperties waveProperties) {
        if(null == waves) {
            waves = new WaveProperties[] { waveProperties };
            waveLayers++;
        }else{
            waves = Arrays.copyOf(waves, waves.length +1);
            waves[waves.length -1] = waveProperties;
            //waveLayers = waves.length;
            waveLayers++;
        }
    }
    
    public static void addWave(float wavelength, float amplitude, float speed, Vector2f direction, boolean isCircular) {
        addWave(new WaveProperties(wavelength, amplitude, speed, direction, isCircular));
    }
    
    public static void addWave(float wavelength, float amplitude, float speed, Vector2f direction) {
        addWave(new WaveProperties(wavelength, amplitude, speed, direction, false));
    }
    
    public static float getHeightAt(float x, float z, float time) {
        if(null == waves || waveLayers < 1) {
            return 0f;
        }
        
        /*
        The Parameters of a Single Wave Function
        Wavelength (L): the crest-to-crest distance between waves in world space. 
            Wavelength L relates to frequency (w) as w = 2/L.
        Amplitude (A): the height from the water plane to the wave crest.
        Speed (S): the distance the crest moves forward per second. 
            It is convenient to express speed as phase-constant φ, where φ = S x 2/L.
        Direction (D): the horizontal vector perpendicular to the wave front along which the crest travels.
        Then the state of each wave as a function of horizontal position (x, z) and time (t) is defined as:

            W(x,z,t) = A * sin(D dot(x,z) * w + t * φ)

        And the total surface is: 

            H(x,z,t) = sum[ Ai * sin(Di dot(x,z) * wi + t * φi) ]

        For choppy waves with exponent (k) = chop:

            W(x,z,t) = 2A * ((sin(D dot(x,z) * w + t * φ) + 1) /2)^(k)
        */
        Vector2f samplePoint = new Vector2f(x, z);
        float netWaveHeight = 0f;
        for(int i =0; i <waveLayers; i++) {
            Vector2f direction = waves[i].getDirection();
            float frequency = waves[i].getFrequency();
            float amplitude = waves[i].getAmplitude();
            float phase = waves[i].getPhase();
            boolean isCircular = waves[i].isIsCircular();
            
            if(isCircular) {
                direction = direction.subtract(samplePoint).normalize();
            }else{
                direction = direction.negate();
            }
            
            float waveHeight;
            float sin = FastMath.sin(direction.dot(samplePoint) * frequency + time * phase);
            if(isChoppy) {
                waveHeight = amplitude * 2 * FastMath.pow((sin + 1) / 2, chop);
            }else{
                waveHeight = amplitude * sin;
            }
            
            netWaveHeight += waveHeight;
        }
        
        return netWaveHeight;
    }
    
    public static Vector3f getNormalAt(float x, float z, float time) {
        if(null == waves || waveLayers < 1) {
            return new Vector3f(0f, 1f, 0f);
        }
        
        /*
        N(x,z) = [ -∂/∂x(H(x,z,t)), -∂/∂z(H(x,z,t)), 1 ]
        
        ∂/∂x(H(x,z,t)) = sum[ wi * Di.x * Ai * cos(Di dot(x,z) * wi + t * φi) ]
        ∂/∂z(H(x,z,t)) = sum[ wi * Di.y * Ai * cos(Di dot(x,z) * wi + t * φi) ]
        over all waves i
        
        For choppy waves:
        
         ∂/∂x(W(x,z,t)) = sum[ k * Di.x * wi * Ai * 
                            ((sin(Di dot(x,z) * wi + t * φi) + 1) /2)^(k-1) * 
                            cos(Di dot(x,z) * wi + t * φi) ]
         ∂/∂y(W(x,z,t)) = sum[ k * Di.y * wi * Ai * 
                            ((sin(Di dot(x,z) * wi + t * φi) + 1) /2)^(k-1) * 
                            cos(Di dot(x,z) * wi + t * φi) ]
        over all waves i
        */
        
        Vector2f samplePoint = new Vector2f(x, z);
        float normalX = 0f;
        float normalZ = 0f;
        for(int i =0; i <waveLayers; i++) {
            Vector2f direction = waves[i].getDirection();
            float frequency = waves[i].getFrequency();
            float amplitude = waves[i].getAmplitude();
            float phase = waves[i].getPhase();
            boolean isCircular = waves[i].isIsCircular();
            
            if(isCircular) {
                direction = direction.subtract(samplePoint).normalize();
            }else{
                direction = direction.negate();
            }
            
            float cos = FastMath.cos(direction.dot(samplePoint) * frequency + phase * time);
            
            // calculate the partial derivatives for x and z to multiply with netWaveHeight
            if(isChoppy){
                float sin = FastMath.sin(direction.dot(samplePoint) * frequency + time * phase); 
                normalX +=
                        chop *
                        frequency *
                        direction.getX() *
                        amplitude *
                        FastMath.pow((sin +1)/2, chop -1) *
                        cos;
                normalZ += 
                        chop *
                        frequency *
                        direction.getY() *
                        amplitude *
                        FastMath.pow((sin +1)/2, chop -1) *
                        cos;
            }else{
                normalX += 
                        frequency *
                        direction.getX() *
                        amplitude *
                        cos;
                normalZ += 
                        frequency *
                        direction.getY() *
                        amplitude *
                        cos;
            }
        }
        return new Vector3f(-normalX, 1, -normalZ).normalizeLocal();
    }
    
    public static Vector3f getTangentAt(float x, float z, float time) {
        if(null == waves || waveLayers < 1) {
            return new Vector3f(1f, 0f, 0f);
        }
        
        /*
            T(x,z) = (0f, ∂/∂z(H(x,z,t)), 1f)
            ∂/∂z(H(x,z,t)) = sum[ wi * Di.y * Ai * cos(Di dot(x,z) * wi + t * φi) ]
            over all waves i

            For choppy waves:
            ∂/∂z(W(x,z,t)) = sum[ k * Di.y * wi * Ai * 
                                    ((sin(Di dot(x,z) * wi + t * φi) + 1) /2)^(k-1) * 
                                    cos(Di dot(x,z) * wi + t * φi) ]
            over all waves i
        */
        Vector2f samplePoint = new Vector2f(x, z);
        float tangentY = 0f;
        for(int i =0; i <waveLayers; i++) {
            Vector2f direction = waves[i].getDirection();
            float frequency = waves[i].getFrequency();
            float amplitude = waves[i].getAmplitude();
            float phase = waves[i].getPhase();
            boolean isCircular = waves[i].isIsCircular();
            
            if(isCircular) {
                direction = direction.subtract(samplePoint).normalize();
            }else{
                direction = direction.negate();
            }
            
            float trigSeed = direction.dot(samplePoint) * frequency + phase * time;
            float cos = FastMath.cos(trigSeed);
            float sin = FastMath.sin(trigSeed);
            
            // calculate the partial derivative w.r.t. z to multiply with netWaveHeight
            if(isChoppy) {
                tangentY += chop * direction.getY() * frequency * amplitude * 
                                FastMath.pow((sin +1)/2, chop -1) * cos;
            }else{
                tangentY += frequency * direction.getY() * amplitude * cos;
            }
        }
        return new Vector3f(0, tangentY, 1).normalizeLocal();
    } 
    
    public static Vector3f getBinormalAt(float x, float z, float time) {
        if(null == waves || waveLayers < 1) {
            return new Vector3f(0f, 0f, 1f);
        }
        
        Vector2f samplePoint = new Vector2f(x, z);
        float binormalY = 0f;
        
        for(int i =0; i <waveLayers; i++) {
            Vector2f direction = waves[i].getDirection();
            float frequency = waves[i].getFrequency();
            float amplitude = waves[i].getAmplitude();
            float phase = waves[i].getPhase();
            boolean isCircular = waves[i].isIsCircular();
            
            if(isCircular) {
                direction = direction.subtract(samplePoint).normalize();
            }else{
                direction = direction.negate();
            }
            
            float trigSeed = direction.dot(samplePoint) * frequency + phase * time;
            float cos = FastMath.cos(trigSeed);
            float sin = FastMath.sin(trigSeed);
            
            // calculate the partial derivative w.r.t. x to multiply with netWaveHeight
            if(isChoppy) {
                binormalY += chop * direction.getX() * frequency * amplitude * 
                                FastMath.pow((sin +1)/2, chop -1) * cos;
            }else{
                binormalY += frequency * direction.getX() * amplitude * cos;
            }
        }
        return new Vector3f(1, binormalY, 0).normalizeLocal();
    }
    
    
    
//    private static float medianWavelength;
//     private static float halfMedianWavelength;
//     private static float doubleMedianWavelength;
//     private static float medianWavelengthOffset;
//    private static float waveFrequency;
//    private static float medianAmplitude;
//    private static Vector2f windDirection;
//    private static float maxAngleOffsetDegrees;
    private static float steepness = 0.9f; // 0.0 to 1.0
    
    public static Vector3f getGerstnerPositionAt(float x, float z, float time) {
        if(null == waves || waveLayers < 1) {
            return new Vector3f(x, 0f, z);
        }
        
        float X = 0f;
        float Y = 0f;
        float Z = 0f; 
        Vector2f P = new Vector2f(x, z);
        
        for(int i=0; i <waveLayers; i++){
            Vector2f waveDirection = waves[i].getDirection();
            float waveX = waveDirection.getX();
            float waveZ = waveDirection.getY();
            float amplitude = waves[i].getAmplitude();
            float frequency = waves[i].getFrequency();
            float phaseT = waves[i].getPhase() * time;
            float dot = waveDirection.dot(P);
            float cos = FastMath.cos(frequency * dot + phaseT);
            float sin = FastMath.sin(frequency * dot + phaseT);
            //steepness = 0f; // 1f/(frequency * amplitude);
            float steepnessLocal = steepness / (frequency * amplitude * waveLayers);
            
            X += steepnessLocal * amplitude * waveX * cos;
            Y += amplitude * sin;
            Z += steepnessLocal * amplitude * waveZ * cos;
        }
        return new Vector3f(X, Y, Z);
    }
    
    @Deprecated // broken
    public static Vector3f getGerstnerPositionAt(float x, float z, float time, boolean deleteMe) {
        /*
        Q = steepness constant {0 to 1}
        P(x,y,t) = (x + sum[Qi * Ai * Di.x * cos(wi * Di dot(x,y) + φ^time)],
                    sum[Ai * sin(wi * Di dot(x,y) + φ^time)],
                    z + sum[Qi * Ai * Di.y * cos(wi * Di dot(x,y) + φ^time)])
        */
        float X = x;
        float Y = 0f;
        float Z = z;
        for(int i=0; i <waveLayers; i++){
            Vector2f waveDirection = waves[i].getDirection();
            float waveX = waveDirection.getX();
            float waveZ = waveDirection.getY();
            float amplitude = waves[i].getAmplitude();
            float frequency = waves[i].getFrequency();
            float phaseT = FastMath.pow(waves[i].getPhase(), time);
            float dot = waveDirection.dot(new Vector2f(x,z));
            float cos = FastMath.cos(frequency * dot + phaseT);
            float sin = FastMath.sin(frequency * dot + phaseT);
            steepness = 1f/(frequency * amplitude);
            
            X += steepness * amplitude * waveX * cos;
            Y += amplitude * sin;
            Z += steepness * amplitude * waveZ * cos;
        }
        return new Vector3f(X, Y, Z);
    }

    public static Vector3f getGerstnerNormalAt(float x, float z, float time) {
        if(null == waves || waveLayers < 1) {
            return new Vector3f(0f, 1f, 0f);
        }
        
        float tempX = 0f;
        float tempY = 0f;
        float tempZ = 0f;
        // WA = wi * Ai
        // C() = cos(wi * Di dot(P) + φ * time)
        // S() = sin(wi * Di dot(P) + φ * time)
        for(int i =0; i <waveLayers; i++) {
            Vector2f waveDirection = waves[i].getDirection();
            float waveX = waveDirection.getX();
            float waveZ = waveDirection.getY();
            float amplitude = waves[i].getAmplitude();
            float frequency = waves[i].getFrequency();
            float phaseT = waves[i].getPhase() * time;
            float dot = waveDirection.dot(new Vector2f(x,z));
            float cos = FastMath.cos(frequency * dot + phaseT);
            float sin = FastMath.sin(frequency * dot + phaseT);
            float wa = frequency * amplitude;
            float steepnessLocal = steepness / (frequency * amplitude * waveLayers);
            // sum[Di.x * WA * C()]
            tempX += waveX * wa * cos;
            // sum[Di.y * WA * C()]
            tempZ += waveZ * wa * cos;  
            // sum[Qi * WA * S()]
            tempY += steepnessLocal * wa * sin;
        }
        return new Vector3f(-tempX, 1 - tempY, -tempZ);
    }
    
    public static Vector3f getGerstnerTangentAt(float x, float z, float time) {
        if(null == waves || waveLayers < 1) {
            return new Vector3f(1f, 0f, 0f);
        }
        
        float tempX = 0f;
        float tempY = 0f;
        float tempZ = 0f;
        // WA = wi * Ai
        // C() = cos(wi * Di dot(P) + φ * time)
        // S() = sin(wi * Di dot(P) + φ * time)
        for(int i =0; i <waveLayers; i++) {
            Vector2f waveDirection = waves[i].getDirection();
            float waveX = waveDirection.getX();
            float waveZ = waveDirection.getY();
            float amplitude = waves[i].getAmplitude();
            float frequency = waves[i].getFrequency();
            float phaseT = waves[i].getPhase() * time;
            float dot = waveDirection.dot(new Vector2f(x,z));
            float cos = FastMath.cos(frequency * dot + phaseT);
            float sin = FastMath.sin(frequency * dot + phaseT);
            float wa = frequency * amplitude;
            float steepnessLocal = steepness / (frequency * amplitude * waveLayers);
            // sum[Qi * Di.x *Di.y * WA * S()]
            tempX += steepnessLocal * waveX * waveZ * wa * sin; 
            // sum[Qi * Di.y^2 * WA * S()]
            tempZ += steepnessLocal * waveZ * waveZ * wa * sin;  
            // sum[Di.y * WA * C()]
            tempY += waveZ * wa * cos;
        }
        return new Vector3f(-tempX, tempY, 1 - tempZ);
    }
    
    public static Vector3f getGerstnerBinormalAt(float x, float z, float time) {
        if(null == waves || waveLayers < 1) {
            return new Vector3f(0f, 0f, 1f);
        }
        
        float tempX = 0f;
        float tempY = 0f;
        float tempZ = 0f;
        // WA = wi * Ai
        // C() = cos(wi * Di dot(P) + φ * time)
        // S() = sin(wi * Di dot(P) + φ * time)
        for(int i =0; i <waveLayers; i++) {
            Vector2f waveDirection = waves[i].getDirection();
            float waveX = waveDirection.getX();
            float waveZ = waveDirection.getY();
            float amplitude = waves[i].getAmplitude();
            float frequency = waves[i].getFrequency();
            float phaseT = waves[i].getPhase() * time;
            float dot = waveDirection.dot(new Vector2f(x,z));
            float cos = FastMath.cos(frequency * dot + phaseT);
            float sin = FastMath.sin(frequency * dot + phaseT);
            float wa = frequency * amplitude;
            float steepnessLocal = steepness / (frequency * amplitude * waveLayers);
            // sum[Qi * Di.x *Di.y * WA * S()]
            tempX += steepnessLocal * waveX * waveZ * wa * sin; 
            // sum[Qi * Di.x^2 * WA * S()]
            tempZ += steepnessLocal * waveX * waveX * wa * sin;  
            // sum[Di.x * WA * C()]
            tempY += waveX * wa * cos;
        }
        return new Vector3f(1 - tempX, tempY, -tempZ);
    }
       
    @Deprecated // not accurate
    public static float getGerstnerHeightAt(float x, float z, float time) {
        return getGerstnerPositionAt(x, z, time).getY();
    }
    
    @Deprecated // broken
    public static float getGerstnerHeightAt(float x, float z, float time, boolean deleteMe) {
        // may be an issue using z loc as 2d y input, negate z input?
        Vector3f firstResult = getGerstnerPositionAt(x, z, time);
        float xOffset = x - firstResult.getX();
        float zOffset = z - firstResult.getZ();
        Vector3f secondResult = getGerstnerPositionAt(xOffset, zOffset, time);
        return secondResult.getY();
    }
    
    @Deprecated // broken
    public static Vector2f getGerstnerOffsetAt(float x, float z, float time) {
        // may be an issue using z loc as 2d y input, negate z input?
        Vector3f firstResult = getGerstnerPositionAt(x, z, time);
        return new Vector2f(firstResult.getX(), firstResult.getZ());
    }
    

    public static float dot(float ax, float az, float bx, float bz) {
        return ax * bx + az * bz;
    }
    
}
