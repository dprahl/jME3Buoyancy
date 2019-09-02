package bouyancy.surface;

import com.jme3.math.FastMath;
import com.jme3.math.Vector2f;

/**
 *
 * @author User
 */
public class WaveProperties {
        private float wavelength;
        private float frequency;
        private float amplitude;
        private float speed;
        private float phase;
        private Vector2f direction;
        private boolean isCircular;
        
        /**
         * @param wavelength the distance from crest to crest 
         * @param amplitude the height from the water plane to the wave crest
         * @param speed the distance the crest moves forward per second
         * @param direction the direction perpendicular to the wave front 
         *          and the direction of travel (will be normalized)
         *          presumed to be in the horizontal plane of x and z
         *          Vector2f.y will store the z component
         */
        public WaveProperties(float wavelength, float amplitude, float speed, Vector2f direction) {
            this(wavelength, amplitude, speed, direction, false);
        }
        
        /**
         * @param wavelength the distance from crest to crest 
         * @param amplitude the height from the water plane to the wave crest
         * @param speed the distance the crest moves forward per second
         * @param direction the direction perpendicular to the wave front 
         *          and the direction of travel (will be normalized if isCircular is set to false)
         *          presumed to be in the horizontal plane of x and z
         *          Vector2f.y will store the z component
         * @param isCircular true if wave is to be circular, false for directional
         */
        public WaveProperties(float wavelength, float amplitude, float speed, Vector2f direction, boolean isCircular) {
            this.wavelength = wavelength;
            //this.frequency = 2/wavelength; // upgraded
            this.frequency = FastMath.sqrt(9.98f *(FastMath.TWO_PI/wavelength));
            this.amplitude = amplitude;
            this.speed = speed;
            this.phase = speed * frequency;
            this.direction = (isCircular ? direction : direction.normalize());
            this.isCircular = isCircular;
        }
        
        /**
         * @param wavelength the distance from crest to crest 
         * @param amplitude the height from the water plane to the wave crest
         * @param speed the distance the crest moves forward per second
         * @param direction the direction perpendicular to the wave front 
         *          and the direction of travel (will be normalized)
         *          presumed to be in the horizontal plane of x and z
         *          Vector2f.y will store the z component
         */
        public void setProperties(float wavelength, float amplitude, float speed, Vector2f direction) {
            setProperties(wavelength, amplitude, speed, direction, false);
        }
        
        /**
         * @param wavelength the distance from crest to crest 
         * @param amplitude the height from the water plane to the wave crest
         * @param speed the distance the crest moves forward per second
         * @param direction the direction perpendicular to the wave front 
         *          and the direction of travel (will be normalized if isCircular is set false)
         *          presumed to be in the horizontal plane of x and z
         *          Vector2f.y will store the z component
         * @param isCircular true if wave is to be circular, false for directional
         */
        public void setProperties(float wavelength, float amplitude, float speed, Vector2f direction, boolean isCircular) {
            this.wavelength = wavelength;
            //this.frequency = 2/wavelength; // upgraded
            this.frequency = FastMath.sqrt(9.98f *(FastMath.TWO_PI/wavelength));
            this.amplitude = amplitude;
            this.speed = speed;
            this.phase = speed * frequency;
            this.direction = (isCircular ? direction : direction.normalize());
            this.isCircular = isCircular;
        }
        
        /**
         * @return the wavelength, the distance from crest to crest 
         */
        public float getWavelength() {
            return wavelength;
        }

        /**
         * @param wavelength the wavelength to set
         */
        public void setWavelength(float wavelength) {
            this.wavelength = wavelength;
        }

        /**
         * @return the frequency of the wave
         */
        public float getFrequency() {
            return frequency;
        }

        /**
         * @param frequency the frequency to assign to the wave,
         * internal debug use only
         * frequency is initialized to be 2/wavelength
         */
        @Deprecated
        private void setFrequency(float frequency) {
            this.frequency = frequency;
        }

        /**
         * @return the amplitude, the height from 
         * the water plane to the wave crest
         */
        public float getAmplitude() {
            return amplitude;
        }

        /**
         * @param amplitude the amplitude to set
         */
        public void setAmplitude(float amplitude) {
            this.amplitude = amplitude;
        }

        /**
         * @return the speed of the wave front,
         * the distance the crest moves forward per second
         */
        public float getSpeed() {
            return speed;
        }

        /**
         * @param speed the speed to set
         */
        public void setSpeed(float speed) {
            this.speed = speed;
        }

        /**
         * @return the phase
         */
        public float getPhase() {
            return phase;
        }

        /**
         * @param phase the phase to set, 
         * for internal debug use only
         */
        @Deprecated
        private void setPhase(float phase) {
            this.phase = phase;
        }

        /**
         * @return the 2d direction (normalized) of 
         * the x and z components of the wave
         *   Vector2f.getY() will return the z component
         */
        public Vector2f getDirection() {
            return direction;
        }

        /**
         * @param direction the direction perpendicular to the wave front 
         * and the direction of travel (will be normalized)
         * presumed to be in the horizontal plane of x and z
         *   Vector2f.y will store the z component
         */
        public void setDirection(Vector2f direction) {
            this.direction = direction.normalize();
        }
        
        /**
         * @return the 2d center point of
         * the source of the circular wave
         * presumed to be in the horizontal plane of x and z
         *   Vector2f.getY() will return the z component
         */
        public Vector2f getCenter() {
            return direction;
        }
        
        /**
         * @param center the 2d center location of
         * the source of the circular wave
         * presumed to be in the horizontal plane of x and z
         *   Vector2f.y will store the z component
         */
        public void setCenter(Vector2f center) {
            this.direction = center;
        }
        
        /**
         * @return true if wave is Circular, false if wave is Directional
         */
        public boolean isIsCircular() {
            return isCircular;
        }
        
        /**
         * @param isCircular true if wave is to be circular
         *  use {@link #setCenter(Vector2f) setCenter} to ensure position is not normalized
         */
        public void setIsCircular(boolean isCircular) {
            this.isCircular = isCircular;
        }

    }